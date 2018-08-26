#include "nest.h"

static void clock_setup(void) {
    rcc_clock_setup_in_hsi_out_48mhz();

    /* Enable GPIOC clock for LED & USARTs. */
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Enable clocks for USART2. */
    rcc_periph_clock_enable(RCC_USART1);

    // Enable clock for SPI
    rcc_periph_clock_enable(RCC_SPI1);
}

static void i2c_setup(void) {
    // Ensure clock is enabled
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_set_i2c_clock_hsi(I2C1);

    // Reset any existing i2c conf
    i2c_reset(I2C1);

    // Configure GPIOs
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);

    i2c_peripheral_disable(I2C1);

    i2c_enable_analog_filter(I2C1);
    i2c_set_digital_filter(I2C1, 0);

    i2c_set_speed(I2C1, i2c_speed_sm_100k, 48);

    i2c_set_7bit_addr_mode(I2C1);
    i2c_peripheral_enable(I2C1);
}

static void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
    STK_CVR = 0;
    systick_set_reload(rcc_ahb_frequency / 8 / 1000);
    systick_counter_enable();
}

static void gpio_setup(void) {
    /* Setup GPIO pin GPIO8/9 on GPIO port C for LEDs. */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9);

    /* Setup GPIO pins for USART2 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
}

#define ADDR 0x60

int main(void)
{
    clock_setup();
    systick_setup();
    gpio_setup();
    uart_setup();
    uart_putln("\r\nBoot");

    i2c_setup();

    // Done
    while (1) {
    }

    return 0;
}
