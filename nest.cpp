#include "nest.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

extern "C" {
    #include "winc1500/socket/include/socket.h"
    #include "winc1500/driver/include/m2m_periph.h"
    #include "winc1500/driver/include/m2m_wifi.h"
    #include "winc1500/driver/include/m2m_ssl.h"
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
    Wifi.deep_sleep_mode_enable();
}

#define READ_0_RATE_MS 3000
#define READ_1_RATE_MS 30000

uint64_t last_read_0 = 0;
uint64_t last_read_1 = 0;
uint64_t last_packet = 0;

WifiFsm wifi_fsm;
    double temp, rh;

void nest_event_loop() {
    if (millis() - last_read_0 > READ_0_RATE_MS) {
        // printf("Sensor 0: ");
        if (sht_read(SHT_0_ADDR, &temp, &rh)) {
            // printf("Temp: %.2f, Rel humidity: %.2f%%\n", temp, rh);
            wifi_fsm.send_temperature(temp);
            wifi_fsm.send_rh(rh);
        } else {
            // printf("READ ERROR\n");
        }
        last_read_0 = millis();
    }
    if (millis() - last_read_1 > READ_1_RATE_MS) {
        // printf("Sensor 1: ");
        if (sht_read(SHT_1_ADDR, &temp, &rh)) {
            //printf("Temp: %.2f, Rel humidity: %.2f%%\n", temp, rh);
        } else {
            //printf("READ ERROR\n");
        }
        last_read_1 = millis();
    }

    // Handle wifi events
    wifi_fsm.event_loop();
}
