#include "nest_wifi_manager.h"

// Callback bridges
static void n_wifi_handle_event(uint8_t message_type, void *message_data);
static void n_wifi_socket_cb(SOCKET sock, uint8_t message_type, void *message_data);
static void n_wifi_resolv_cb(uint8_t *host_name, uint32_t host_info);

static void n_wifi_handle_event(uint8_t message_type, void *message_data) {
    Wifi.handle_event(message_type, message_data);
}

static void n_wifi_socket_cb(SOCKET sock, uint8_t message_type, void *message_data) {
    Wifi.handle_socket_event(sock, message_type, message_data);
}

static void n_wifi_resolv_cb(uint8_t *host_name, uint32_t host_info) {
    Wifi.handle_resolve(host_name, host_info);
}

WifiMgr::WifiMgr() {
}

uint8_t WifiMgr::init() {
    // If already initialized, nothing to do
    if (_initialized) {
        return 0;
    }

    n_log("Initializing WiFi... ");

    // Initialize board support package
    nm_bsp_init();

    // Initialize callback
    tstrWifiInitParam param;
    param.pfAppWifiCb = n_wifi_handle_event;
    int8_t ret = m2m_wifi_init(&param);

    // Check inititalization succeeded
    // If it didn't log and return error
    if (ret != M2M_SUCCESS) {
        printf("failed!\n");

        if (ret == M2M_ERR_FW_VER_MISMATCH) {
            n_log("WiFi firmware version mismatch\n");
        } else {
            n_log("WiFi init error: %x\n", ret);
        }

        return ret;
    }

#ifdef CONF_PERIPH
    // Init LEDS.
    // GPIO4 = wifi, 5 = activity, 6 = error
    m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 1);
    m2m_periph_gpio_set_val(M2M_PERIPH_GPIO5, 1);
    m2m_periph_gpio_set_val(M2M_PERIPH_GPIO6, 1);
    m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO4, 1);
    m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO5, 1);
    m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO6, 1);
#endif

    // Enable extra ciphers for firmware > 19.5.0
    tstrM2mRev rev;
    nm_get_firmware_info(&rev);
    uint32_t firmware_version = M2M_MAKE_VERSION(rev.u8FirmwareMajor, rev.u8FirmwareMinor, rev.u8FirmwarePatch);
    if (firmware_version >= M2M_MAKE_VERSION(19, 5, 0)) {
        m2m_ssl_set_active_ciphersuites(SSL_NON_ECC_CIPHERS_AES_128 | SSL_NON_ECC_CIPHERS_AES_256);
    }

    // Initialize socket callback
    socketDeinit();
    socketInit();
    registerSocketCallback(n_wifi_socket_cb, n_wifi_resolv_cb);

    printf("success.\n");
    _initialized = true;

    deep_sleep_mode_enable();

    return 0;
}

void WifiMgr::hard_reset() {
    // Put chip back into reset
    n_log("Performing hard-reset of wifi\n");
    nm_bsp_reset();
    _initialized = false;
    _connection_state = M2M_WIFI_UNDEF;
    _has_ip_addr = false;
    _last_ntp_sync = 0;
}

void WifiMgr::led_enable_wifi() { m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 0); }
void WifiMgr::led_enable_act() { m2m_periph_gpio_set_val(M2M_PERIPH_GPIO5, 0); }
void WifiMgr::led_enable_error() { m2m_periph_gpio_set_val(M2M_PERIPH_GPIO6, 0); }
void WifiMgr::led_disable_wifi() { m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 1); }
void WifiMgr::led_disable_act() { m2m_periph_gpio_set_val(M2M_PERIPH_GPIO5, 1); }
void WifiMgr::led_disable_error() { m2m_periph_gpio_set_val(M2M_PERIPH_GPIO6, 1); }

uint8_t WifiMgr::connect(const char *ssid, const char *psk) {
    // Make sure that wifi is actually initialized
    if (!_initialized) {
        init();
    }

    // Begin WPA2 connection
    n_log("Connecting to SSID '%s'\n", ssid);
    int ret = m2m_wifi_connect(
        (char *)ssid, strlen(ssid),
        M2M_WIFI_SEC_WPA_PSK,
        (void *)psk,
        M2M_WIFI_CH_ALL
    );
    if (ret) {
        n_log("Failed to connect: 0x%x\n", ret);
        led_enable_error();
        return ret;
    } else {
        led_disable_error();
    }

    return 0;
}

uint8_t WifiMgr::wait_for_connection() {
    n_log("Waiting up to 60 seconds for WiFi connection...\n");
    const uint64_t until = millis() + 60000;
    while (millis() < until && connection_state() == M2M_WIFI_UNDEF) {
        m2m_wifi_handle_events(NULL);
    }

    if (connection_state() != M2M_WIFI_CONNECTED) {
        n_log("Timed out waiting for connection\n");
        return 1;
    }

    m2m_wifi_get_connection_info();
    m2m_wifi_handle_events(NULL);
    return 0;
}

void WifiMgr::sleep_mode_enable() {
    m2m_wifi_set_sleep_mode(M2M_PS_H_AUTOMATIC, true);
}

void WifiMgr::deep_sleep_mode_enable() {
    m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, true);
}

void WifiMgr::sleep_mode_disable() {
    m2m_wifi_set_sleep_mode(M2M_NO_PS, false);
}

void WifiMgr::handle_event(uint8_t message_type, void *message_data) {
    const char *msg_name = "Unknown";
    switch (message_type) {
        case M2M_WIFI_RESP_CON_STATE_CHANGED:
            // Extra data = tstrM2mWifiStateChanged*
            handle_resp_conn_state_changed(static_cast<tstrM2mWifiStateChanged *>(message_data));
            break;
        case M2M_WIFI_REQ_DHCP_CONF: // DHCP complete, payload is IP address
            // Extra data = IP address, as 4 uint8_t
            handle_req_dhcp_conf(static_cast<uint8_t*>(message_data));
            break;
        case M2M_WIFI_RESP_GET_SYS_TIME:
            handle_resp_get_sys_time(static_cast<tstrSystemTime *>(message_data));
            break;
        case M2M_WIFI_RESP_CONN_INFO:
            handle_resp_conn_info(static_cast<tstrM2MConnInfo *>(message_data));
            break;
        default:
            if (is_m2m_config_cmd(message_type)) {
                msg_name = m2_config_cmd_to_string((tenuM2mConfigCmd) message_type);
            } else if (is_m2m_sta_cmd(message_type)) {
                msg_name = m2_sta_cmd_to_string((tenuM2mStaCmd) message_type);
            }
            n_log("Got unhandled wifi event 0x%x (%s), additional data %p\n",
                message_type, msg_name, message_data);
            break;
    }
}

void WifiMgr::handle_resp_conn_info(tstrM2MConnInfo *info) {
    n_log("Connection state:\nIP Address: %u.%u.%u.%u\nMAC: %02x:%02x:%02x:%02x:%02x:%02x\nRSSI: %d\n",
        info->au8IPAddr[0], info->au8IPAddr[1], info->au8IPAddr[2], info->au8IPAddr[3],
        info->au8MACAddress[0], info->au8MACAddress[1], info->au8MACAddress[2],
        info->au8MACAddress[3], info->au8MACAddress[4], info->au8MACAddress[5],
        info->s8RSSI
    );
}

void WifiMgr::handle_resp_get_sys_time(tstrSystemTime *systime) {
    uint16_t y = systime->u16Year;
    uint8_t m = systime->u8Month;
    uint8_t d = systime->u8Day;
    uint64_t t;

    //January and February are counted as months 13 and 14 of the previous year
    if(m <= 2) {
       m += 12;
       y -= 1;
    }

    //Convert years to days
    t = (365 * y) + (y / 4) - (y / 100) + (y / 400);
    //Convert months to days
    t += (30 * m) + (3 * (m + 1) / 5) + d;
    //Unix time starts on January 1st, 1970
    t -= 719561;
    //Convert days to seconds
    t *= 86400;
    //Add hours, minutes and seconds
    t += (3600 * systime->u8Hour) + (60 * systime->u8Minute) + systime->u8Second;
    // Convert to ms
    t *= 1000;

    // Update our UTC offset
    set_utc_offset(t);

    // Log it
    n_log("UTC offset updated\n");
}

void WifiMgr::handle_resp_conn_state_changed(tstrM2mWifiStateChanged* new_state) {
    n_log("Wifi connection state changed to %s",
        new_state->u8CurrState == M2M_WIFI_CONNECTED ? "connected" : "disconnected");
    if (new_state->u8CurrState == M2M_WIFI_CONNECTED) {
        printf("\n");
        led_enable_wifi();
    } else {
        printf("Error code: %u\n", new_state->u8CurrState);
        led_disable_wifi();
    }
    _connection_state = static_cast<tenuM2mConnState>(new_state->u8CurrState);
}

void WifiMgr::handle_req_dhcp_conf(uint8_t *ip_address) {
    // Copy IP address over
    memcpy(_ip_address, ip_address, 4);
    _has_ip_addr = true;

    // Log
    n_log("Received DHCP lease %u.%u.%u.%u\n",
        ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
}

bool WifiMgr::has_ip_address() {
    return _has_ip_addr;
}

tenuM2mConnState WifiMgr::connection_state() {
    return _connection_state;
}

void WifiMgr::handle_socket_event(SOCKET sock, uint8_t message_type, void *message_data) {
    // If there's a handler for this socket, call that
    if (_socket_handlers[sock]) {
        led_enable_act();
        _socket_handlers[sock]->socket_cb(sock, message_type, message_data);
        led_disable_act();
        return;
    }

    switch(message_type) {
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
            n_log("Connect on socket %d\n", sock);
            break;
        case SOCKET_MSG_RECV:
        case SOCKET_MSG_RECVFROM:
            n_log("Recv on socket %d\n", sock);
            break;
        case SOCKET_MSG_SEND:
        case SOCKET_MSG_SENDTO:
            // n_log("Send on socket %d\n", sock);
            // led_enable_act();
            // led_disable_act();
            break;
    }
}

void WifiMgr::handle_socket_recv(SOCKET sock, tstrSocketRecvMsg *data) {
    if (data->s16BufferSize < 0) {
        sock_close(sock);
        return;
    }

    if (_sockets[sock].state == SOCK_STATE_CONNECTED
            || _sockets[sock].state == SOCK_STATE_BOUND) {
        _sockets[sock].recvMsg.pu8Buffer = data->pu8Buffer;
        _sockets[sock].recvMsg.s16BufferSize = data->s16BufferSize;
        if (sock < TCP_SOCK_MAX) {
            // TCP socket
        }  else {
            // UDP socket
            // Update the remote IP addr
            _sockets[sock].recvMsg.strRemoteAddr = data->strRemoteAddr;
        }
    } else {
        // Discard
        // hif_receive(0, NULL, 0, 1);
    }
}

void WifiMgr::handle_resolve(uint8_t *host_name, uint32_t host_info) {
    for (int i = 0; i < MAX_SOCKET; i++) {
        if (_socket_handlers[i] != NULL) {
            _socket_handlers[i]->resolve_cb(host_name, host_info);
        }
    }
}

bool is_m2m_config_cmd(uint8_t cmd) { return cmd >= M2M_CONFIG_CMD_BASE && cmd < M2M_WIFI_MAX_CONFIG_ALL; }
bool is_m2m_sta_cmd(uint8_t cmd) { return cmd >= M2M_STA_CMD_BASE&& cmd < M2M_WIFI_MAX_STA_ALL;}
bool is_m2m_ap_cmd(uint8_t cmd) { return cmd >= M2M_AP_CMD_BASE&& cmd < M2M_WIFI_MAX_AP_ALL;}
bool is_m2m_p2p_cmd(uint8_t cmd) { return cmd >= M2M_P2P_CMD_BASE&& cmd < M2M_WIFI_MAX_P2P_ALL;}
bool is_m2m_server_cmd(uint8_t cmd) { return cmd >= M2M_SERVER_CMD_BASE && cmd < M2M_WIFI_MAX_SERVER_ALL;}

const char *m2_config_cmd_to_string(tenuM2mConfigCmd cmd) {
    switch (cmd) {
        case M2M_WIFI_REQ_RESTART: return "M2M_WIFI_REQ_RESTART";
        case M2M_WIFI_REQ_SET_MAC_ADDRESS: return "M2M_WIFI_REQ_SET_MAC_ADDRESS";
        case M2M_WIFI_REQ_CURRENT_RSSI: return "M2M_WIFI_REQ_CURRENT_RSSI";
        case M2M_WIFI_RESP_CURRENT_RSSI: return "M2M_WIFI_RESP_CURRENT_RSSI";
        case M2M_WIFI_REQ_GET_CONN_INFO: return "M2M_WIFI_REQ_GET_CONN_INFO";
        case M2M_WIFI_RESP_CONN_INFO: return "M2M_WIFI_RESP_CONN_INFO";
        case M2M_WIFI_REQ_SET_DEVICE_NAME: return "M2M_WIFI_REQ_SET_DEVICE_NAME";
        case M2M_WIFI_REQ_START_PROVISION_MODE: return "M2M_WIFI_REQ_START_PROVISION_MODE";
        case M2M_WIFI_RESP_PROVISION_INFO: return "M2M_WIFI_RESP_PROVISION_INFO";
        case M2M_WIFI_REQ_STOP_PROVISION_MODE: return "M2M_WIFI_REQ_STOP_PROVISION_MODE";
        case M2M_WIFI_REQ_SET_SYS_TIME: return "M2M_WIFI_REQ_SET_SYS_TIME";
        case M2M_WIFI_REQ_ENABLE_SNTP_CLIENT: return "M2M_WIFI_REQ_ENABLE_SNTP_CLIENT";
        case M2M_WIFI_REQ_DISABLE_SNTP_CLIENT: return "M2M_WIFI_REQ_DISABLE_SNTP_CLIENT";
        case M2M_WIFI_RESP_MEMORY_RECOVER: return "M2M_WIFI_RESP_MEMORY_RECOVER";
        case M2M_WIFI_REQ_CUST_INFO_ELEMENT: return "M2M_WIFI_REQ_CUST_INFO_ELEMENT";
        case M2M_WIFI_REQ_SCAN: return "M2M_WIFI_REQ_SCAN";
        case M2M_WIFI_RESP_SCAN_DONE: return "M2M_WIFI_RESP_SCAN_DONE";
        case M2M_WIFI_REQ_SCAN_RESULT: return "M2M_WIFI_REQ_SCAN_RESULT";
        case M2M_WIFI_RESP_SCAN_RESULT: return "M2M_WIFI_RESP_SCAN_RESULT";
        case M2M_WIFI_REQ_SET_SCAN_OPTION: return "M2M_WIFI_REQ_SET_SCAN_OPTION";
        case M2M_WIFI_REQ_SET_SCAN_REGION: return "M2M_WIFI_REQ_SET_SCAN_REGION";
        case M2M_WIFI_REQ_SET_POWER_PROFILE: return "M2M_WIFI_REQ_SET_POWER_PROFILE";
        case M2M_WIFI_REQ_SET_TX_POWER: return "M2M_WIFI_REQ_SET_TX_POWER";
        case M2M_WIFI_REQ_SET_BATTERY_VOLTAGE: return "M2M_WIFI_REQ_SET_BATTERY_VOLTAGE";
        case M2M_WIFI_REQ_SET_ENABLE_LOGS: return "M2M_WIFI_REQ_SET_ENABLE_LOGS";
        case M2M_WIFI_REQ_GET_SYS_TIME: return "M2M_WIFI_REQ_GET_SYS_TIME";
        case M2M_WIFI_RESP_GET_SYS_TIME: return "M2M_WIFI_RESP_GET_SYS_TIME";
        case M2M_WIFI_REQ_SEND_ETHERNET_PACKET: return "M2M_WIFI_REQ_SEND_ETHERNET_PACKET";
        case M2M_WIFI_RESP_ETHERNET_RX_PACKET: return "M2M_WIFI_RESP_ETHERNET_RX_PACKET";
        case M2M_WIFI_REQ_SET_MAC_MCAST: return "M2M_WIFI_REQ_SET_MAC_MCAST";
        case M2M_WIFI_REQ_GET_PRNG: return "M2M_WIFI_REQ_GET_PRNG";
        case M2M_WIFI_RESP_GET_PRNG: return "M2M_WIFI_RESP_GET_PRNG";
        case M2M_WIFI_REQ_SCAN_SSID_LIST: return "M2M_WIFI_REQ_SCAN_SSID_LIST";
        case M2M_WIFI_REQ_SET_GAINS: return "M2M_WIFI_REQ_SET_GAINS";
        case M2M_WIFI_REQ_PASSIVE_SCAN: return "M2M_WIFI_REQ_PASSIVE_SCAN";
        default: return "M2M_UNKNOWN";
    }
}

const char *m2_sta_cmd_to_string(tenuM2mStaCmd cmd) {
    switch (cmd) {
        case M2M_WIFI_REQ_CONNECT: return "M2M_WIFI_REQ_CONNECT";
        case M2M_WIFI_REQ_DEFAULT_CONNECT: return "M2M_WIFI_REQ_DEFAULT_CONNECT";
        case M2M_WIFI_RESP_DEFAULT_CONNECT: return "M2M_WIFI_RESP_DEFAULT_CONNECT";
        case M2M_WIFI_REQ_DISCONNECT: return "M2M_WIFI_REQ_DISCONNECT";
        case M2M_WIFI_RESP_CON_STATE_CHANGED: return "M2M_WIFI_RESP_CON_STATE_CHANGED";
        case M2M_WIFI_REQ_SLEEP: return "M2M_WIFI_REQ_SLEEP";
        case M2M_WIFI_REQ_WPS_SCAN: return "M2M_WIFI_REQ_WPS_SCAN";
        case M2M_WIFI_REQ_WPS: return "M2M_WIFI_REQ_WPS";
        case M2M_WIFI_REQ_START_WPS: return "M2M_WIFI_REQ_START_WPS";
        case M2M_WIFI_REQ_DISABLE_WPS: return "M2M_WIFI_REQ_DISABLE_WPS";
        case M2M_WIFI_REQ_DHCP_CONF: return "M2M_WIFI_REQ_DHCP_CONF";
        case M2M_WIFI_RESP_IP_CONFIGURED: return "M2M_WIFI_RESP_IP_CONFIGURED";
        case M2M_WIFI_RESP_IP_CONFLICT: return "M2M_WIFI_RESP_IP_CONFLICT";
        case M2M_WIFI_REQ_ENABLE_MONITORING: return "M2M_WIFI_REQ_ENABLE_MONITORING";
        case M2M_WIFI_REQ_DISABLE_MONITORING: return "M2M_WIFI_REQ_DISABLE_MONITORING";
        case M2M_WIFI_RESP_WIFI_RX_PACKET: return "M2M_WIFI_RESP_WIFI_RX_PACKET";
        case M2M_WIFI_REQ_SEND_WIFI_PACKET: return "M2M_WIFI_REQ_SEND_WIFI_PACKET";
        case M2M_WIFI_REQ_LSN_INT: return "M2M_WIFI_REQ_LSN_INT";
        case M2M_WIFI_REQ_DOZE: return "M2M_WIFI_REQ_DOZE";
        default: return "M2M_UNKNOWN";
    }
}

void WifiMgr::register_socket_handler(SOCKET sock, WifiFsm *fsm) {
    _socket_handlers[sock] = fsm;
}

void WifiMgr::unregister_socket_handler(SOCKET sock, WifiFsm *fsm) {
    if (_socket_handlers[sock] == fsm) {
        _socket_handlers[sock] = NULL;
    }
}

void WifiMgr::event_loop() {
    // Process any pending events in the driver
    m2m_wifi_handle_events(NULL);

    // Maybe update system time sync
    update_system_time();

}

void WifiMgr::update_system_time() {
    if (millis() - _last_ntp_sync < NTP_SYNC_INTERVAL_MS) {
        return;
    }
    if (M2M_SUCCESS != m2m_wifi_get_sytem_time()) {
        n_log("Failed to request system time\n");
    }
    _last_ntp_sync = millis();
}

WifiMgr Wifi;
