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
class Sensors Sensors;

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
#ifdef WIFI_FIRMWARE_UPDATE_MODE
    wifi_firmware_update();
#endif
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
#ifndef WIFI_FIRMWARE_UPDATE_MODE
    iwdg_start();
#endif

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
    lcd.powerOn();

    // Enable wifi
    Wifi.init();
}

uint64_t last_print = 0;
void nest_event_loop() {
    // Update temp, RH, brightness
    Sensors.update();

    // Refresh display
    lcd.update();

    // Handle base wifi events
    Wifi.event_loop();

    // Handle socket events
    wifi_fsm.event_loop();

    // Tickle the watchdog
    iwdg_reset();

    // Uart liveness indicator
    if (millis() - last_print > 1000) {
        last_print = millis();
        const time_t now = n_utc() / 1000;
        struct tm local = *localtime(&now);
        char buf[20];
        strftime(buf, 21, "%Y-%m-%d  %H:%M:%S", &local);
        printf("[%s]\r", buf);

        uint8_t backlight = 128 - (128 * (Sensors.get_brightness() / 100.0f));
        if (backlight < 32)
            backlight = 0;
        lcd.set_backlight(backlight);
    }
}
