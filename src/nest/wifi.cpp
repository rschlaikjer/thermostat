#include <nest/wifi.hpp>

// Handle any events that need doing
void WifiFsm::event_loop() {
  // Ensure we're connected to wifi
  ensure_wifi_connected();

  // Make sure that our socket exists
  ensure_socket_connected();

  // Check if we need to send a heartbeat
  check_and_send_heartbeat();

  // Handle any received data
  process_received_data();
}

void WifiFsm::ensure_wifi_connected() {
  // Check if we're connected
  if (Wifi.connection_state() == M2M_WIFI_CONNECTED) {
    // All good
    return;
  }

  // If we aren't check how long it's been since we last tried to connect
  const uint64_t time_since_last_connect = millis() - _last_wifi_connect_start;

  // If we're still in a connect cooldown, come back later
  if (time_since_last_connect < RECONNECT_DELAY_MS) {
    return;
  }

  // If we're not, initiate another connect request
  Wifi.connect(N_SECRET_WIFI_SSID, N_SECRET_WIFI_PSK);
  _last_wifi_connect_start = millis();
}

void WifiFsm::resolve_cb(uint8_t *hostname, uint32_t ip_addr) {
  // If this isn't the hostname we wanted to resolve, ignore it
  if (strcmp((char *)hostname, N_SECRET_SERVER_HOSTNAME) != 0) {
    return;
  }

  // If it is, update our state
  _resolved_hostname = true;
  _hostname_is_resolving = false;
  _resolved_ip = _ntohl(ip_addr);
  n_log("Resolved '%s' -> %lu.%lu.%lu.%lu\n", hostname, (_resolved_ip >> 24),
        ((_resolved_ip >> 16) & 0xFF), ((_resolved_ip >> 8) & 0xFF),
        _resolved_ip & 0xFF);
}

void WifiFsm::socket_cb(SOCKET sock, uint8_t evt, void *evt_data) {
  // If this data isn't for us, ignore
  if (_sock != sock) {
    n_log("Got unexpected event for socket %d (expected %d)\n", sock, _sock);
    return;
  }

  // Connect data
  tstrSocketConnectMsg *connect_status;
  tstrSocketRecvMsg *recv_msg;
  int16_t i;
  switch (evt) {
  case SOCKET_MSG_BIND:
    n_log("Bound socket %d\n", sock);
    break;
  case SOCKET_MSG_LISTEN:
    n_log("Listen socket %d\n", sock);
    break;
  case SOCKET_MSG_DNS_RESOLVE:
    n_log("DNS resolv on socket %d\n", sock);
    break;
  case SOCKET_MSG_ACCEPT:
    n_log("Accept on socket %d\n", sock);
    break;
  case SOCKET_MSG_CONNECT:
    connect_status = (tstrSocketConnectMsg *)evt_data;
    if (connect_status->s8Error == SOCK_ERR_NO_ERROR) {
      // Mark socket as bound
      _sock_bound = true;

      // Send a hello
      send_hello();
    } else {
      // Failed to bind, reset binding state so we try again
      n_log("Failed to bind socket %d: %s\n", sock,
            socket_error_str(connect_status->s8Error));
      _sock_bound = false;
      _sock_is_binding = false;
    }
    break;
  case SOCKET_MSG_RECV:
  case SOCKET_MSG_RECVFROM:
    // n_log("Recv on socket %d\n", sock);
    recv_msg = static_cast<tstrSocketRecvMsg *>(evt_data);
    if (recv_msg->s16BufferSize > 0) {
      // If there's more data to be read, immediately re-schedule
      // the recv call
      if (recv_msg->u16RemainingSize > 0) {
        _last_recv_poll = 0;
      }
    } else if (recv_msg->s16BufferSize == SOCK_ERR_TIMEOUT) {
      // No data to recv
    } else if (recv_msg->s16BufferSize < 0) {
      n_log("Failed to recv data on sock %d: %s\n", sock,
            socket_error_str(recv_msg->s16BufferSize));
      reset_socket();
    }
    break;
  case SOCKET_MSG_SEND:
  case SOCKET_MSG_SENDTO:
    i = *reinterpret_cast<int16_t *>(evt_data);
    if (i < 0) {
      n_log("Failed send on sock %d: %s\n", sock, socket_error_str(i));
      reset_socket();
    }
    break;
  }
}

void WifiFsm::ensure_socket_connected() {
  // If our socket is non-zero, all good
  if (_sock >= 0 && _sock_bound) {
    return;
  }

  // If we're not connected to wifi, can't make a socket yet
  if (Wifi.connection_state() != M2M_WIFI_CONNECTED || !Wifi.has_ip_address()) {
    return;
  }

  // If we don't have a socket, make that
  if (_sock < 0) {
    _sock = socket(AF_INET, SOCK_STREAM, 0);
    _sock_bound = false;
    _sock_is_binding = false;
    _resolved_hostname = false;
    _hostname_is_resolving = false;
    if (_sock < 0) {
      // Error creating socket
      n_log("Failed to create socket: %s\n", socket_error_str(_sock));
      return;
    }
    n_log("Opened TCP socket %d\n", _sock);

    // Register ourselves with the wifi manager for socket callbacks
    Wifi.register_socket_handler(_sock, this);
  }

  // Before we can connect, we need to resolve the hostname
  if (!_resolved_hostname) {
    if (!_hostname_is_resolving) {
      int8_t resolv_result = gethostbyname((uint8_t *)N_SECRET_SERVER_HOSTNAME);
      if (resolv_result < 0) {
        // If we fail to resolve the name, there's not much we can do
        // other than try to reset the wifi connection
        n_log("Failed to resolve hostname %s: %s\n", N_SECRET_SERVER_HOSTNAME,
              socket_error_str(resolv_result));
        _resolved_hostname = false;
        _hostname_is_resolving = false;
        reset_socket();
        Wifi.hard_reset();
      } else {
        n_log("Resolving '%s'\n", N_SECRET_SERVER_HOSTNAME);
        _resolved_hostname = false;
        _hostname_is_resolving = true;
        _hostname_resolve_start = millis();
      }
    } else {
      // Hostname still resolving - time out after 10s
      if (millis() - _hostname_resolve_start > HOSTNAME_RESOLV_TIMEOUT) {
        n_log("DNS resolve timed out\n");
        _resolved_hostname = false;
        _hostname_is_resolving = false;
      }
    }
  }

  // Initiate a connection on the socket, if we're not bound yet
  if (_resolved_hostname && !_sock_bound && !_sock_is_binding) {
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = _htons(N_SECRET_SERVER_PORT);
    addr.sin_addr.s_addr = _htonl(_resolved_ip);
    n_log("Connecting to %lu.%lu.%lu.%lu:%u\n", _resolved_ip >> 24,
          (_resolved_ip >> 16) & 0xFF, (_resolved_ip >> 8) & 0xFF,
          _resolved_ip & 0xFF, N_SECRET_SERVER_PORT);
    sint8 err = connect(_sock, (struct sockaddr *)&addr, sizeof(addr));

    // Check if we got a connection error
    if (err < 0) {
      n_log("Failed to connect socket: %s\n", socket_error_str(err));
      sock_close(_sock);
      _sock = -1;
      return;
    } else {
      // Set the sock bind state to in-progress
      _sock_is_binding = true;
    }
  }
}

const char *socket_error_str(int error) {
  switch (error) {
  case SOCK_ERR_NO_ERROR:
    return "SOCK_ERR_NO_ERROR";
  case SOCK_ERR_INVALID_ADDRESS:
    return "SOCK_ERR_INVALID_ADDRESS";
  case SOCK_ERR_ADDR_ALREADY_IN_USE:
    return "SOCK_ERR_ADDR_ALREADY_IN_USE";
  case SOCK_ERR_MAX_TCP_SOCK:
    return "SOCK_ERR_MAX_TCP_SOCK";
  case SOCK_ERR_MAX_UDP_SOCK:
    return "SOCK_ERR_MAX_UDP_SOCK";
  case SOCK_ERR_INVALID_ARG:
    return "SOCK_ERR_INVALID_ARG";
  case SOCK_ERR_MAX_LISTEN_SOCK:
    return "SOCK_ERR_MAX_LISTEN_SOCK";
  case SOCK_ERR_INVALID:
    return "SOCK_ERR_INVALID";
  case SOCK_ERR_ADDR_IS_REQUIRED:
    return "SOCK_ERR_ADDR_IS_REQUIRED";
  case SOCK_ERR_CONN_ABORTED:
    return "SOCK_ERR_CONN_ABORTED";
  case SOCK_ERR_TIMEOUT:
    return "SOCK_ERR_TIMEOUT";
  case SOCK_ERR_BUFFER_FULL:
    return "SOCK_ERR_BUFFER_FULL";
  default:
    return "SOCK_ERROR_UNKNOWN";
  }
}

void WifiFsm::send_temperature(double temp) {
  sock_send((uint8_t)CMD_TEMP);
  sock_send((float)temp);
}

void WifiFsm::send_rh(double rh) {
  sock_send((uint8_t)CMD_RH);
  sock_send((float)rh);
}

void WifiFsm::send_brightness(float brightness) {
  sock_send((uint8_t)CMD_LIGHT);
  sock_send((float)brightness);
}

void WifiFsm::send_uptime(uint64_t uptime) {
  sock_send((uint8_t)CMD_UPTIME);
  sock_send(uptime);
}

void WifiFsm::send_log_msg(const char *msg, int len) {
  return;
  if (len <= 0)
    return;
  if (len > 0xFFFF)
    return;
  if (_sock < 0 || !_sock_bound)
    return;
  uint8_t prelude[3];
  prelude[0] = CMD_LOG;
  prelude[1] = (((uint16_t)len) >> 8) & 0xFF;
  prelude[2] = ((uint16_t)len) & 0xFF;
  if (send(_sock, prelude, 3, 0))
    return;
  send(_sock, (void *)msg, len, 0);
}

void WifiFsm::send_hello() {
  // Send the protocol knock
  sock_send("NEST");
  sock_send((uint8_t)NEST_PROTOCOL_VER);

  // Send the node ID
  sock_send((uint8_t)CMD_NODE_ID);
  sock_send((uint8_t)12); // Number of bytes in ID
  sock_send(reinterpret_cast<uint8_t *>(STM32F0_UUID_ADDR), 12);
}

void WifiFsm::sock_send(float v) {
  unsigned char *v_bytes = (unsigned char *)&v;
  sock_send(v_bytes, sizeof(float));
}

void WifiFsm::sock_send(double v) {
  unsigned char *v_bytes = (unsigned char *)&v;
  sock_send(v_bytes, sizeof(double));
}

void WifiFsm::sock_send(uint32_t v) {
  sock_send(reinterpret_cast<uint8_t *>(&v), sizeof(uint32_t));
}

void WifiFsm::sock_send(uint64_t v) {
  sock_send(reinterpret_cast<uint8_t *>(&v), sizeof(uint64_t));
}

void WifiFsm::sock_send(const char *str) {
  sock_send((uint8_t *)str, strlen(str));
}

void WifiFsm::sock_send(uint8_t c) { sock_send(&c, 1); }

void WifiFsm::sock_send(uint8_t *data, size_t bytes) {
  // If the socket isn't connected, just drop data
  if (_sock < 0 || !_sock_bound) {
    n_log("Socket down, dropping data\r");
    // If it's been down for some time, force a reset
    if (millis() - _last_data_sent > SOCKET_MAX_DOWNTIME_MS) {
      reset_socket();
    }
    return;
  }

  int16_t ret = send(_sock, data, bytes, 0); // Flags field unused
  if (ret < 0) {
    n_log("Failed to send data: %s\n", socket_error_str(ret));
    reset_socket();
  } else {
    _last_data_sent = millis();
  }
}

void WifiFsm::reset_socket() {
  n_log("Resetting socket\n");
  if (_sock >= 0) {
    n_log("Closing socket %d\n", _sock);
    Wifi.unregister_socket_handler(_sock, this);
    sock_close(_sock);
  }
  _sock = -1;
  _sock_bound = false;
  _sock_is_binding = false;
  _resolved_hostname = false;
  _hostname_is_resolving = false;
  // Update this value so we don't spin if this is the reset reason
  _last_data_sent = millis();
}

void WifiFsm::check_and_send_heartbeat() {}

void WifiFsm::process_received_data() {
  // If socket not connected, do nothing
  if (_sock < 0 || !_sock_bound) {
    return;
  }

  // Only poll for recv data every now and again
  const uint64_t time_since_last_poll = millis() - _last_recv_poll;
  if (time_since_last_poll < RECV_POLL_RATE_MS) {
    return;
  }

  int16_t ret = recv(_sock, _recv_buffer, 256, 100);
  _last_recv_poll = millis();
  if (ret < 0) {
    n_log("Failed to recv data: %s\n", socket_error_str(ret));
  }
}

WifiFsm wifi_fsm;
