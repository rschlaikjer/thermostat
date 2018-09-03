#ifndef NEST_WIFI_H
#define NEST_WIFI_H

#include <string.h>

extern "C" {
#include "winc1500/socket/include/socket.h"
#include "winc1500/driver/include/m2m_periph.h"
#include "winc1500/driver/include/m2m_wifi.h"
#include "winc1500/driver/include/m2m_ssl.h"
}

#include "nest_uart.h"
#include "nest_realtime.h"
#include "nest_secrets.h"

typedef enum {
    WL_NO_SHIELD = 255,
    WL_IDLE_STATUS = 0,
    WL_NO_SSID_AVAIL,
    WL_SCAN_COMPLETED,
    WL_CONNECTED,
    WL_CONNECT_FAILED,
    WL_CONNECTION_LOST,
    WL_DISCONNECTED,
    WL_AP_LISTENING,
    WL_AP_CONNECTED,
    WL_AP_FAILED,
    WL_PROVISIONING,
    WL_PROVISIONING_FAILED
} wl_status_t;

typedef enum {
    WL_RESET_MODE = 0,
    WL_STA_MODE,
    WL_PROV_MODE,
    WL_AP_MODE
} wl_mode_t;

class WifiMgr {
    public:
        WifiMgr();

        uint8_t init();
        uint8_t connect(const char *ssid, const char *psk);
        uint8_t wait_for_connection();
        tenuM2mConnState connection_state();

        void handle_event(uint8_t message_type, void *message_data);
        void handle_socket_event(SOCKET sock, uint8_t message_type, void *message_data);
        void handle_resolve(uint8_t *host_name, uint32_t host_info);
    private:
        bool _initialized = false;
        tenuM2mConnState _connection_state = M2M_WIFI_UNDEF;
        uint8_t _ip_address[4];

        // Status LED management
        void led_enable_wifi();
        void led_enable_act();
        void led_enable_error();
        void led_disable_wifi();
        void led_disable_act();
        void led_disable_error();

        // Individual callback handlers
        void handle_resp_conn_state_changed(tstrM2mWifiStateChanged* new_state);
        void handle_req_dhcp_conf(uint8_t *ip_address);
        void handle_resp_get_sys_time(tstrSystemTime *systime);
};

bool is_m2m_config_cmd(uint8_t cmd);
bool is_m2m_sta_cmd(uint8_t cmd);
bool is_m2m_ap_cmd(uint8_t cmd);
bool is_m2m_p2p_cmd(uint8_t cmd);
bool is_m2m_server_cmd(uint8_t cmd);

const char *m2_sta_cmd_to_string(tenuM2mStaCmd cmd);
const char *m2_config_cmd_to_string(tenuM2mConfigCmd cmd);

extern WifiMgr Wifi;

#endif // NEST_WIFI_H
