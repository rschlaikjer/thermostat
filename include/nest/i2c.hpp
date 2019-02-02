#ifndef NEST_I2C_H
#define NEST_I2C_H

#include <stdio.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>

#include <nest/realtime.hpp>
#include <nest/uart.hpp>

#define NEST_I2C I2C1

#define NEST_I2C_XFER_OK 0x01
#define NEST_I2C_ERROR   0x00

// Initialize the i2c bus
void n_i2c_setup(void);
void i2c_scan(void);

// Perform a transfer over i2c with the given address.
// Returns 1 on success, 0 on error.
uint8_t n_i2c_transfer(const uint8_t address,
                       uint8_t *write, size_t write_count,
                       uint8_t *read, size_t read_count);

#endif // NEST_I2C_H
