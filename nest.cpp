#include "nest.h"

static void clock_setup(void) {
    // Set clock at 48MHz from internal oscillator
    rcc_clock_setup_in_hsi_out_48mhz();

    // Enable GPIO clocks
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // Enable clock for SPI
    rcc_periph_clock_enable(RCC_SPI1);
}


static void gpio_setup(void) {
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO9 | GPIO10);
}

#define ADDR 0x60

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
    gpio_setup();
    uart_setup();
    n_i2c_setup();
    adc_setup();
}

void nest_event_loop() {
    sht_log();
    uart_putln("");
    uint16_t light = adc_read();
    uart_puts("Brightness: ");
    uart_putd(light);
    uart_putln("");
    sleep(3000);
}
