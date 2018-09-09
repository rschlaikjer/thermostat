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

LCD lcd;

static void clock_setup(void) {
    // Set clock at 48MHz from internal oscillator
    rcc_clock_setup_in_hsi_out_48mhz();

    // Enable LSI clock for IWDG
    rcc_osc_on(RCC_LSI);

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
    // Configure system clock
    clock_setup();

    // Configure and enable watchdog timer
    iwdg_set_period_ms(WATCHDOG_TIMEOUT_MS);
    iwdg_reset();
    // iwdg_start();

    // Enable systick to provide real-time-ish clocl
    systick_setup();

    // Enable serial console
    uart_setup();

    // Log if we got restarted by IWDG
    if (RCC_CSR & RCC_CSR_IWDGRSTF) {
        n_log("Restarted by watchdog\n");
    }

    // Clear the reset cause register
    RCC_CSR |= RCC_CSR_RMVF;

    n_log("Unique chip ID: 0x%08lx%08lx%08lx\n",
        STM32F0_CHIP_ID[0], STM32F0_CHIP_ID[1], STM32F0_CHIP_ID[2]);

    // Check the IWDG state
    uint32_t *option_data = reinterpret_cast<uint32_t*>(0x1FFFF800);
    n_log("Option bytes:\n", *option_data);
    for (uint8_t i = 0; i < 4; i++) {
        n_log("    0x%08lx: %08lx\n",
            option_data + sizeof(uint32_t) * i,
            option_data[i]);
    }
    bool hw_iwdg_enabled = (
        *option_data & (1 << 24) && !(*option_data & (1 << 16))
    );
    n_log("Hardware watchdog ");
    printf(hw_iwdg_enabled ? "enabled\n" : "DISABLED\n");

    // Initialize peripherals
    interrupt_init();
    n_i2c_setup();
    n_spi_setup();
    adc_setup();
    n_relay_init();
    lcd.init();

    // Enable wifi
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
        if (sht_read(SHT_1_ADDR, &temp, &rh)) {
            wifi_fsm.send_temperature(temp);
            wifi_fsm.send_rh(rh);
        }

        uint16_t brightness = adc_read();
        wifi_fsm.send_brightness(brightness);

        last_read_0 = millis();
    }

    lcd.update();

    // Handle base wifi events
    Wifi.event_loop();

    // Handle socket events
    wifi_fsm.event_loop();

    // Tickle the watchdog
    iwdg_reset();
}
