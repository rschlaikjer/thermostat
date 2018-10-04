#ifndef NEST_WIFI_MANAGER_H
#define NEST_WIFI_MANAGER_H

#include <string.h>

extern "C" {
#include "winc1500/driver/include/m2m_periph.h"
#include "winc1500/driver/include/m2m_wifi.h"
#include "winc1500/driver/include/m2m_ssl.h"
#include "winc1500/socket/include/m2m_socket_host_if.h"
#include "winc1500/socket/include/socket.h"
}

#include "nest_realtime.h"
#include "nest_wifi.h"
#include "nest_secrets.h"

#define SOCKET_BUFFER_SIZE 1472
#define NTP_SYNC_INTERVAL_MS 3600000 // 1 hr

typedef enum {
    WL_RESET_MODE = 0,
    WL_STA_MODE,
    WL_PROV_MODE,
    WL_AP_MODE
} wl_mode_t;

typedef enum {
    SOCK_STATE_INVALID,
    SOCK_STATE_IDLE,
    SOCK_STATE_CONNECTING,
    SOCK_STATE_CONNECTED,
    SOCK_STATE_BINDING,
    SOCK_STATE_BOUND,
    SOCK_STATE_LISTEN,
    SOCK_STATE_LISTENING,
    SOCK_STATE_ACCEPTED
} wl_socket_state;

typedef struct {
    wl_socket_state state;
    tstrSocketRecvMsg recvMsg;
    struct {
        uint8_t *data;
        uint8_t *head;
        int length;
    } buffer;
} wl_socket_info;

typedef void (*socket_handler_fun)(SOCKET socket, uint8_t cmd, void *cmd_data);

class WifiFsm;

class WifiMgr {
    public:
        WifiMgr();

        uint8_t init();
        uint8_t connect(const char *ssid, const char *psk);
        uint8_t wait_for_connection();
        tenuM2mConnState connection_state();
        bool has_ip_address();

        void handle_event(uint8_t message_type, void *message_data);
        void handle_socket_event(SOCKET sock, uint8_t message_type, void *message_data);
        void handle_resolve(uint8_t *host_name, uint32_t host_info);

        void sleep_mode_enable();
        void deep_sleep_mode_enable();
        void sleep_mode_disable();

        void event_loop();

        void register_socket_handler(SOCKET sock, WifiFsm *fsm);
        void unregister_socket_handler(SOCKET sock, WifiFsm *fsm);

        void hard_reset();

    private:
        bool _initialized = false;
        tenuM2mConnState _connection_state = M2M_WIFI_UNDEF;
        uint8_t _ip_address[4];
        bool _has_ip_addr = false;
        wl_socket_info _sockets[MAX_SOCKET];
        WifiFsm* _socket_handlers[MAX_SOCKET] = {NULL};
        uint64_t _last_ntp_sync = 0;

        // Status LED management
        void led_enable_wifi();
        void led_enable_error();
        void led_enable_act();
        void led_disable_wifi();
        void led_disable_error();
        void led_disable_act();

        void update_system_time();

        // Individual callback handlers
        void handle_resp_conn_state_changed(tstrM2mWifiStateChanged* new_state);
        void handle_req_dhcp_conf(uint8_t *ip_address);
        void handle_resp_get_sys_time(tstrSystemTime *systime);
        void handle_resp_conn_info(tstrM2MConnInfo * message_data);

        // Socket callback handlers
        void handle_socket_recv(SOCKET sock, tstrSocketRecvMsg* message_data);
};

bool is_m2m_config_cmd(uint8_t cmd);
bool is_m2m_sta_cmd(uint8_t cmd);
bool is_m2m_ap_cmd(uint8_t cmd);
bool is_m2m_p2p_cmd(uint8_t cmd);
bool is_m2m_server_cmd(uint8_t cmd);

const char *m2_sta_cmd_to_string(tenuM2mStaCmd cmd);
const char *m2_config_cmd_to_string(tenuM2mConfigCmd cmd);

extern WifiMgr Wifi;

#endif // NEST_WIFI_MANAGER_H
