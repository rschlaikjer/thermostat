#ifndef NEST_LCD_H
#define NEST_LCD_H

#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#define NEST_I2C I2C1

#define LCD_I2C_ADDR 0x72

#endif // NEST_LCD_H
