#ifndef NEST_WIFI_H
#define NEST_WIFI_H

#include "nest_realtime.h"
#include "nest_wifi_manager.h"
#include "nest_secrets.h"

#define RECONNECT_DELAY_MS 60000
#define RECV_POLL_RATE_MS 1000
#define RECV_BUFFER_SIZE 128

#define STM32F0_UUID_ADDR 0x1FFFF7AC

typedef enum {
    CMD_HELLO = 1,
    CMD_TEMP,
    CMD_LIGHT,
    CMD_TARGET_TEMP,
    CMD_ZONE_ACTIVE,
    CMD_RH
} wifi_fsm_cmd;

class WifiFsm {

    public:
        void event_loop();

        void send_temperature(double temp);
        void send_rh(double rh);

        void socket_cb(SOCKET sock, uint8_t evt, void *evt_data);
    private:
        SOCKET _sock = -1;
        bool _sock_bound = false;
        bool _sock_is_binding = false;
        uint64_t _last_wifi_connect_start = -RECONNECT_DELAY_MS;
        uint64_t _last_recv_poll = 0;
        uint8_t _recv_buffer[RECV_BUFFER_SIZE];

        void ensure_wifi_connected();
        void ensure_socket_connected();
        void reset_socket();
        void check_and_send_heartbeat();
        void process_received_data();

        void send_hello();
        void sock_send(float v);
        void sock_send(double v);
        void sock_send(uint32_t v);
        void sock_send(uint8_t c);
        void sock_send(uint8_t *data, size_t bytes);
};

const char* socket_error_str(int error);
uint32_t ip_addr(uint8_t ip_0, uint8_t ip_1, uint8_t ip_2, uint8_t ip_3);

#endif // NEST_WIFI_H
