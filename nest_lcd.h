#ifndef NEST_LCD_H
#define NEST_LCD_H

#include <stdio.h>
#include <string.h>
#include <time.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "nest_i2c.h"
#include "nest_realtime.h"
#include "nest_sht.h"
#include "nest_adc.h"

#define LCD_I2C_ADDR 0x3C

#define LCD_UPDATE_MS 1000

#define LCD_WIDTH 128
#define LCD_HEIGHT 64

class LCD {
    public:
        void init();
        void update();
    private:
        uint8_t _pixels[LCD_WIDTH * LCD_HEIGHT / 8];
        uint64_t _last_display_update = -LCD_UPDATE_MS;

        // Clear pixel buffer
        void clear();

        // Render null-terminated text at x,y
        void draw_text(const char *str, uint8_t x, uint8_t y);

        // Render bitmap of dimens w,h at x,y
        void draw_icon(const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t x, uint8_t y);

        // Push pixel buffer to LCD
        void render();

        // Set the backlight brightness
        void set_backlight(uint8_t brightness);
};

#endif // NEST_LCD_H
