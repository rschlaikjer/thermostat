#include "nest_i2c.h"

void n_i2c_setup(void) {
    uart_puts("Initializing i2c... ");
    // Ensure clock is enabled
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_set_i2c_clock_hsi(NEST_I2C);

    // Reset any existing i2c conf
    i2c_reset(NEST_I2C);

    // Configure GPIOs
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);

    i2c_peripheral_disable(NEST_I2C);

    i2c_enable_analog_filter(NEST_I2C);
    i2c_set_digital_filter(NEST_I2C, 0);

    i2c_set_speed(NEST_I2C, i2c_speed_sm_100k, 48);

    i2c_set_7bit_addr_mode(NEST_I2C);
    i2c_enable_stretching(NEST_I2C);
    i2c_peripheral_enable(NEST_I2C);

    uart_putln("done.");
}

uint8_t n_i2c_transfer(const uint8_t address,
                       uint8_t *write, size_t write_count,
                       uint8_t *read, size_t read_count) {
    // Set the address in question
    i2c_set_7bit_address(NEST_I2C, address);

    // If we have data to write:
    if (write_count) {
        // Set write mode
        i2c_set_write_transfer_dir(NEST_I2C);

        // Set the number of bytes to write
        i2c_set_bytes_to_transfer(NEST_I2C, write_count);

        // If we need to read data as well, don't auto-end the i2c transfer
        if (read_count) {
            i2c_disable_autoend(NEST_I2C);
        } else {
            i2c_enable_autoend(NEST_I2C);
        }

        // Start the transfer
        i2c_send_start(NEST_I2C);

        // For each byte to write:
        while (write_count--) {

            // Wait until we are either clear to send or get a NAK
            bool wait = true;
            while (wait) {
                // Check if clear to send
                if (i2c_transmit_int_status(NEST_I2C)) {
                    // If so, can stop waiting
                    wait = false;
                }

                // If there's been a NAK, then abort
                if (i2c_nack(NEST_I2C)) {
                    uart_puts("NAK while trying to communicate with ");
                    uart_putx(address);
                    uart_putln("");
                    return NEST_I2C_ERROR;
                }
            }

            // If the wait is over, send the data
            i2c_send_data(NEST_I2C, *write++);
        }
    }

    // If we have data to read:
    if (read_count) {
        // Set read mode
        i2c_set_read_transfer_dir(NEST_I2C);

        // Set the number of bytes to read
        i2c_set_bytes_to_transfer(NEST_I2C, read_count);

        // Send start condition
        i2c_send_start(NEST_I2C);

        // Safe to auto-end after this
        i2c_enable_autoend(NEST_I2C);

        while (read_count--) {
            *read++ = i2c_get_data(NEST_I2C);
        }
    }

    return NEST_I2C_XFER_OK;
}
