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


static void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
    STK_CVR = 0;
    systick_set_reload(rcc_ahb_frequency / 8 / 1000);
    systick_counter_enable();
}

static void gpio_setup(void) {
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

    sht_log();
}

void nest_event_loop() {

}
