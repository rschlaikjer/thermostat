#include "nest_wifi.h"

// Handle any events that need doing
void WifiFsm::event_loop() {
    // Process any pending events in the driver
    m2m_wifi_handle_events(NULL);

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

void WifiFsm::socket_cb(SOCKET sock, uint8_t evt, void *evt_data) {
    // If this data isn't for us, ignore
    if (_sock != sock) {
        printf("Got unexpected event for socket %d (expected %d)\n", sock, _sock);
        return;
    }

    // Connect data
    tstrSocketConnectMsg* connect_status;

    switch (evt) {
        case SOCKET_MSG_BIND:
            printf("Bound socket %d\n", sock);
            break;
        case SOCKET_MSG_LISTEN:
            printf("Listen socket %d\n", sock);
            break;
        case SOCKET_MSG_DNS_RESOLVE:
            printf("DNS resolv on socket %d\n", sock);
            break;
        case SOCKET_MSG_ACCEPT:
            printf("Accept on socket %d\n", sock);
            break;
        case SOCKET_MSG_CONNECT:
            connect_status = (tstrSocketConnectMsg *) evt_data;
            if (connect_status->s8Error == SOCK_ERR_NO_ERROR) {
                // Mark socket as bound
                _sock_bound = true;

                // Send a hello
                send_hello();
            } else {
                // Failed to bind, reset binding state so we try again
                printf("Failed to bind socket %d: %s\n", sock,
                    socket_error_str(connect_status->s8Error));
                _sock_bound = false;
                _sock_is_binding = false;
            }
            break;
        case SOCKET_MSG_RECV:
        case SOCKET_MSG_RECVFROM:
            printf("Recv on socket %d\n", sock);
            break;
        case SOCKET_MSG_SEND:
        case SOCKET_MSG_SENDTO:
            // printf("Send on socket %d\n", sock);
            break;
    }
}

void WifiFsm::ensure_socket_connected() {
    // If our socket is non-zero, all good
    if (_sock >= 0 && _sock_bound) {
        return;
    }

    // If we're not connected to wifi, can't make a socket yet
    if (Wifi.connection_state() != M2M_WIFI_CONNECTED
            || !Wifi.has_ip_address()) {
        return;
    }

    // If we don't have a socket, make that
    if (_sock < 0) {
        _sock = socket(AF_INET, SOCK_STREAM, 0);
        _sock_bound = false;
        _sock_is_binding = false;
        if (_sock < 0) {
            // Error creating socket
            printf("Failed to create socket: %s\n", socket_error_str(_sock));
            return;
        }

        // Register ourselves with the wifi manager for socket callbacks
        Wifi.register_socket_handler(_sock, this);
    }

    // Initiate a connection on the socket, if we're not bound yet
    if (!_sock_bound && !_sock_is_binding) {
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = _htons(N_SECRET_SERVER_PORT);
        addr.sin_addr.s_addr = _htonl(N_SECRET_SERVER_IP);
        sint8 err = connect(_sock, (struct sockaddr *)&addr, sizeof(addr));

        // Check if we got a connection error
        if (err < 0) {
            printf("Failed to connect socket: %s\n", socket_error_str(err));
            sock_close(_sock);
            _sock = -1;
            return;
        } else {
            // Set the sock bind state to in-progress
            _sock_is_binding = true;
        }
    }
}

uint32_t ip_addr(uint8_t ip_0, uint8_t ip_1, uint8_t ip_2, uint8_t ip_3) {
    return ip_0 << 24 | ip_1 << 16 | ip_2 << 8 | ip_3;
}

const char* socket_error_str(int error) {
    switch (error) {
        case SOCK_ERR_NO_ERROR: return "SOCK_ERR_NO_ERROR";
        case SOCK_ERR_INVALID_ADDRESS: return "SOCK_ERR_INVALID_ADDRESS";
        case SOCK_ERR_ADDR_ALREADY_IN_USE: return "SOCK_ERR_ADDR_ALREADY_IN_USE";
        case SOCK_ERR_MAX_TCP_SOCK: return "SOCK_ERR_MAX_TCP_SOCK";
        case SOCK_ERR_MAX_UDP_SOCK: return "SOCK_ERR_MAX_UDP_SOCK";
        case SOCK_ERR_INVALID_ARG: return "SOCK_ERR_INVALID_ARG";
        case SOCK_ERR_MAX_LISTEN_SOCK: return "SOCK_ERR_MAX_LISTEN_SOCK";
        case SOCK_ERR_INVALID: return "SOCK_ERR_INVALID";
        case SOCK_ERR_ADDR_IS_REQUIRED: return "SOCK_ERR_ADDR_IS_REQUIRED";
        case SOCK_ERR_CONN_ABORTED: return "SOCK_ERR_CONN_ABORTED";
        case SOCK_ERR_TIMEOUT: return "SOCK_ERR_TIMEOUT";
        case SOCK_ERR_BUFFER_FULL: return "SOCK_ERR_BUFFER_FULL";
        default: return "SOCK_ERROR_UNKNOWN";
    }
}

void WifiFsm::send_temperature(double temp) {
    sock_send((uint8_t) CMD_TEMP);
    sock_send((float) temp);
}

void WifiFsm::send_rh(double rh) {
    sock_send((uint8_t) CMD_RH);
    sock_send((float) rh);
}

void WifiFsm::send_hello() {
    sock_send((uint8_t) CMD_HELLO);
    // sock_send((uint8_t) 12); // Number of bytes in ID
    // sock_send(reinterpret_cast<uint8_t*>(STM32F0_UUID_ADDR), 12);
    sock_send(reinterpret_cast<uint8_t*>(STM32F0_UUID_ADDR), 10);
}

void WifiFsm::sock_send(float v) {
    unsigned char *v_bytes = (unsigned char*) &v;
    sock_send(v_bytes, sizeof(float));
}

void WifiFsm::sock_send(double v) {
    unsigned char *v_bytes = (unsigned char*) &v;
    sock_send(v_bytes, sizeof(double));
}

void WifiFsm::sock_send(uint32_t v) {
    sock_send(reinterpret_cast<uint8_t*>(&v), sizeof(uint32_t));
}

void WifiFsm::sock_send(uint8_t c) {
    sock_send(&c, 1);
}

void WifiFsm::sock_send(uint8_t *data, size_t bytes) {
    // If the socket isn't connected, just drop data
    if ( _sock < 0) {
        printf("Socket down, dropping data\n");
        return;
    }

    int16_t ret = send(_sock, data, bytes, 0); // Flags field unused
    if (ret < 0) {
        printf("Failed to send data: %s\n", socket_error_str(ret));
        reset_socket();
    }
}

void WifiFsm::reset_socket() {
    if (_sock >= 0) {
        Wifi.unregister_socket_handler(_sock, this);
        sock_close(_sock);
    }
    _sock = -1;
}

void WifiFsm::check_and_send_heartbeat() {}
void WifiFsm::process_received_data() {}

/*
    if (millis() - last_packet > 10000) {
        if (sock >= 0) {
            double temp, rh;
            if (sht_read(SHT_0_ADDR, &temp, &rh)) {
                Wifi.led_enable_act();
                char payload[256];
                sprintf(payload, "local.temp %f", temp);
                int16_t ret = send(sock, (void *)payload, strlen(payload), 0);
                if (ret < 0) {
                    printf("Failed to send packet: %d\n", ret);
                }
                sprintf(payload, "local.rh %f", rh);
                ret = send(sock, (void *)payload, strlen(payload), 0);
                if (ret < 0) {
                    printf("Failed to send packet: %d\n", ret);
                }
                Wifi.led_disable_act();
                count++;
                printf("Packet count: %llu\n", count);
            }
        } else if (Wifi.connection_state() == M2M_WIFI_CONNECTED) {
            sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock >= 0) {
                printf("Connected socket %d\n", sock);
                addr.sin_family = AF_INET;
                addr.sin_port = _htons(9001);
                // addr.sin_addr.s_addr = _htonl(ip_addr(10, 107, 130, 12));
                addr.sin_addr.s_addr = _htonl(ip_addr(192, 168, 0, 41));
                int ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
                if (ret < 0) {
                    printf("Failed to connect sock: %d\n", ret);
                }
            } else {
                printf("Failed to create socket\n");
            }
        }
        last_packet = millis();
    }
}
*/
