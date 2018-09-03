#include "nest.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

extern "C" {
    #include "winc1500/socket/include/socket.h"
    #include "winc1500/driver/include/m2m_periph.h"
    #include "winc1500/driver/include/m2m_wifi.h"
    #include "winc1500/driver/include/m2m_ssl.h"
    int _write(int file, char *ptr, int len);
}

static void clock_setup(void) {
    // Set clock at 48MHz from internal oscillator
    rcc_clock_setup_in_hsi_out_48mhz();

    // Enable GPIO clocks
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    // Enable GPIO interrupt clock
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
}

int _write(int file, char *ptr, int len)
{
    int i;

    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        for (i = 0; i < len; i++) {
            if (ptr[i] == '\n') {
                usart_send_blocking(NEST_UART, '\r');
            }
            usart_send_blocking(NEST_UART, ptr[i]);
        }
        return i;
    }
    errno = EIO;
    return -1;
}

int main(void) {
    nest_init();
    while (true) {
        nest_event_loop();
    }
    return 0;
}

void nest_init() {
    clock_setup();
    systick_setup();
    uart_setup();

    printf("Unique chip ID: 0x%08lx%08lx%08lx\n",
        STM32F0_CHIP_ID[0], STM32F0_CHIP_ID[1], STM32F0_CHIP_ID[2]);

    n_i2c_setup();
    n_spi_setup();
    adc_setup();

    Wifi.init();
    Wifi.connect(N_SECRET_WIFI_SSID, N_SECRET_WIFI_PSK);
    Wifi.wait_for_connection();
    Wifi.deep_sleep_mode_enable();
}

#define READ_0_RATE_MS 3000
#define READ_1_RATE_MS 30000

uint64_t last_read_0 = 0;
uint64_t last_read_1 = 0;
uint64_t last_packet = 0;

struct sockaddr_in addr;
SOCKET sock;

uint32_t ip_addr(uint8_t ip_0, uint8_t ip_1, uint8_t ip_2, uint8_t ip_3) {
    return ip_0 << 24 | ip_1 << 16 | ip_2 << 8 | ip_3;
}

void nest_event_loop() {
    m2m_wifi_handle_events(NULL);
    if (millis() - last_packet > 10000) {
        if (sock > 0) {
            double temp, rh;
            if (sht_read(SHT_0_ADDR, &temp, &rh)) {
                Wifi.led_enable_act();
                addr.sin_family = AF_INET;
                addr.sin_port = _htons(2003);
                addr.sin_addr.s_addr = _htonl(ip_addr(10, 107, 130, 12));
                // addr.sin_addr.s_addr = _htonl(ip_addr(192, 168, 0, 41));
                char payload[256];
                sprintf(payload, "local.temp %f", temp);
                sendto(sock, (void *)payload, strlen(payload), 0, (struct sockaddr *)&addr, sizeof(addr));
                sprintf(payload, "local.rh %f", rh);
                sendto(sock, (void *)payload, strlen(payload), 0, (struct sockaddr *)&addr, sizeof(addr));
                Wifi.led_disable_act();
            }
        } else if (Wifi.connection_state() == M2M_WIFI_CONNECTED) {
            sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock > 0) {
                printf("Connected socket %d\n", sock);
            } else {
                printf("Failed to create socket\n");
            }
        }
        last_packet = millis();
    }
}

void nest_event_loop_old() {
    double temp, rh;
    if (millis() - last_read_0 > READ_0_RATE_MS) {
        uart_puts("Sensor 0: ");
        if (sht_read(SHT_0_ADDR, &temp, &rh)) {
            uart_puts("Temp: ");
            uart_putf(temp, 2);
            uart_puts(" RH: ");
            uart_putf(rh, 1);
            uart_putln("%");
        } else {
            uart_putln("READ ERROR");
        }
        last_read_0 = millis();
    }
    if (millis() - last_read_1 > READ_1_RATE_MS) {
        uart_puts("Sensor 1: ");
        if (sht_read(SHT_1_ADDR, &temp, &rh)) {
            uart_puts("Temp: ");
            uart_putf(temp, 2);
            uart_puts(" RH: ");
            uart_putf(rh, 1);
            uart_putln("%");
        } else {
            uart_putln("READ ERROR");
        }
        last_read_1 = millis();
    }

}
