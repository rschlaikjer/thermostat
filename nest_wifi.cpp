#include "nest_wifi.h"

uint32_t _resolve;
wl_status_t _status;
wl_mode_t _mode;
int _dhcp;
uint32_t _localip;
uint32_t _submask;
uint32_t _gateway;

char _ssid[M2M_MAX_SSID_LEN];
uint8_t *_remoteMacAddress;

wl_status_t wifi_status() {
    return _status;
}

void n_wifi_handle_event(uint8_t u8MsgType, void *pvMsg) {
    uart_puts("Evt: ");
    uart_putx(u8MsgType);
    uart_putln("");
    switch (u8MsgType) {
        case M2M_WIFI_RESP_DEFAULT_CONNECT:
        {
            uart_putln("M2M_WIFI_RESP_DEFAULT_CONNECT");
            tstrM2MDefaultConnResp *pstrDefaultConnResp = (tstrM2MDefaultConnResp *)pvMsg;
            if (pstrDefaultConnResp->s8ErrorCode) {
                _status = WL_DISCONNECTED;
            }
        }
        break;

        case M2M_WIFI_RESP_CON_STATE_CHANGED:
        {
            uart_putln("M2M_WIFI_RESP_CON_STATE_CHANGED");
            tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
            if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
                uart_putln("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED");
                if (_mode == WL_STA_MODE && !_dhcp) {
                    _status = WL_CONNECTED;

#ifdef CONF_PERIPH
                    // WiFi led ON.
                    m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 0);
#endif
                } else if (_mode == WL_AP_MODE) {
                    _status = WL_AP_CONNECTED;
                }
            } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
                uart_putln("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED");
                if (_mode == WL_STA_MODE) {
                    _status = WL_DISCONNECTED;
                    if (_dhcp) {
                        _localip = 0;
                        _submask = 0;
                        _gateway = 0;
                    }
                    // Close sockets to clean state
                    // Clients will need to reconnect once the physical link will be re-established
                    for (int i = 0; i < MAX_SOCKET; i++) {
                        // WiFiSocket.close(i);
                    }
                } else if (_mode == WL_AP_MODE) {
                    _status = WL_AP_LISTENING;
                }
#ifdef CONF_PERIPH
                // WiFi led OFF (rev A then rev B).
                m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 1);
                m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 1);
#endif
            }
        }
        break;

        case M2M_WIFI_REQ_DHCP_CONF:
        {
            uart_putln("M2M_WIFI_REQ_DHCP_CONF:");
            if (_mode == WL_STA_MODE) {
                tstrM2MIPConfig *pstrIPCfg = (tstrM2MIPConfig *)pvMsg;
                _localip = pstrIPCfg->u32StaticIP;
                _submask = pstrIPCfg->u32SubnetMask;
                _gateway = pstrIPCfg->u32Gateway;

                _status = WL_CONNECTED;

#ifdef CONF_PERIPH
                // WiFi led ON (rev A then rev B).
                m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, 0);
                m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 0);
#endif
            }
            /*uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
            SERIAL_PORT_MONITOR.print("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is ");
            SERIAL_PORT_MONITOR.print(pu8IPAddress[0], 10);
            SERIAL_PORT_MONITOR.print(".");
            SERIAL_PORT_MONITOR.print(pu8IPAddress[1], 10);
            SERIAL_PORT_MONITOR.print(".");
            SERIAL_PORT_MONITOR.print(pu8IPAddress[2], 10);
            SERIAL_PORT_MONITOR.print(".");
            SERIAL_PORT_MONITOR.print(pu8IPAddress[3], 10);
            SERIAL_PORT_MONITOR.println("");*/
        }
        break;

        case M2M_WIFI_RESP_CURRENT_RSSI:
        {
            uart_putln("M2M_WIFI_RESP_CURRENT_RSSI");
            _resolve = *((int8_t *)pvMsg);
        }
        break;

        case M2M_WIFI_RESP_PROVISION_INFO:
        {
            uart_putln("M2M_WIFI_RESP_PROVISION_INFO");
            tstrM2MProvisionInfo *pstrProvInfo = (tstrM2MProvisionInfo *)pvMsg;
            //SERIAL_PORT_MONITOR.println("wifi_cb: M2M_WIFI_RESP_PROVISION_INFO");

            if (pstrProvInfo->u8Status == M2M_SUCCESS) {
                memset(_ssid, 0, M2M_MAX_SSID_LEN);
                memcpy(_ssid, (char *)pstrProvInfo->au8SSID, strlen((char *)pstrProvInfo->au8SSID));
                _mode = WL_STA_MODE;
                _localip = 0;
                _submask = 0;
                _gateway = 0;
                m2m_wifi_connect((char *)pstrProvInfo->au8SSID, strlen((char *)pstrProvInfo->au8SSID),
                        pstrProvInfo->u8SecType, pstrProvInfo->au8Password, M2M_WIFI_CH_ALL);
            } else {
                _status = WL_PROVISIONING_FAILED;
                //SERIAL_PORT_MONITOR.println("wifi_cb: Provision failed.\r\n");
                // beginProvision();
            }
        }
        break;

        case M2M_WIFI_RESP_SCAN_DONE:
        {
            uart_putln("M2M_WIFI_RESP_SCAN_DONE");
            tstrM2mScanDone *pstrInfo = (tstrM2mScanDone *)pvMsg;
            if (pstrInfo->u8NumofCh >= 1) {
                uart_puts("Found ");
                uart_putd(pstrInfo->u8NumofCh);
                uart_putln(" APs");
                m2m_wifi_req_scan_result(0);
                _status = WL_SCAN_COMPLETED;
            }
        }
        break;

        case M2M_WIFI_RESP_SCAN_RESULT:
        {
            uart_putln("Scan result");
            tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult *)pvMsg;
            uint16_t scan_ssid_len = strlen((const char *)pstrScanResult->au8SSID);
            char _scan_ssid[M2M_MAX_SSID_LEN];
            memset(_scan_ssid, 0, M2M_MAX_SSID_LEN);
            if (scan_ssid_len) {
             memcpy(_scan_ssid, (const char *)pstrScanResult->au8SSID, scan_ssid_len);
            }
            uart_puts("Scan: ");
            uart_putln(_scan_ssid);
            if (_remoteMacAddress) {
             // reverse copy the remote MAC
             for(int i = 0; i < 6; i++) {
                 _remoteMacAddress[i] = pstrScanResult->au8BSSID[5-i];
             }
            }
            _resolve = pstrScanResult->s8rssi;
            // _scan_auth = pstrScanResult->u8AuthType;
            // _scan_channel = pstrScanResult->u8ch;
            _status = WL_SCAN_COMPLETED;
        }
        break;

        case M2M_WIFI_RESP_CONN_INFO:
        {
            uart_putln("M2M_WIFI_RESP_CONN_INFO:");
            tstrM2MConnInfo *pstrConnInfo = (tstrM2MConnInfo*)pvMsg;

            if (_remoteMacAddress) {
                // reverse copy the remote MAC
                for(int i = 0; i < 6; i++) {
                    _remoteMacAddress[i] = pstrConnInfo->au8MACAddress[5-i];
                }
                _remoteMacAddress = 0;
            }

            strcpy((char *)_ssid, pstrConnInfo->acSSID);
        }
        break;

        case M2M_WIFI_RESP_GET_SYS_TIME:
        {
            uart_putln("M2M_WIFI_RESP_GET_SYS_TIME::");
            if (_resolve != 0) {
                memcpy((tstrSystemTime *)_resolve, pvMsg, sizeof(tstrSystemTime));

                _resolve = 0;
            }
        }
        break;

        default:
        break;
    }
}
