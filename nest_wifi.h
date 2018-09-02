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

void n_wifi_handle_event(uint8_t u8MsgType, void *pvMsg);
wl_status_t wifi_status(void);

#endif // NEST_WIFI_H
