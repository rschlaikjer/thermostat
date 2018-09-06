#ifndef NEST_LCD_H
#define NEST_LCD_H

#include <stdio.h>
#include <string.h>
#include <time.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "nest_i2c.h"
#include "nest_realtime.h"
#include "nest_sht.h"
#include "nest_adc.h"

#define LCD_I2C_ADDR 0x3C

#define LCD_UPDATE_MS 1000

void lcd_init(void);
void lcd_rom_select(uint8_t rom);
void lcd_update(void);
void lcd_clear(void);

#endif // NEST_LCD_H
