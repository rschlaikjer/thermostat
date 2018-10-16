#ifndef NEST_LCD_H
#define NEST_LCD_H

#include <stdio.h>
#include <string.h>
#include <time.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "nest_spi.h"
#include "nest_realtime.h"
#include "nest_sht.h"
#include "nest_adc.h"

#define LCD_UPDATE_MS 1000

#define LCD_WIDTH 128
#define LCD_HEIGHT 64

#define LCD_PORT_RESET GPIOA
#define LCD_PIN_RESET GPIO8
#define LCD_PORT_CS GPIOB
#define LCD_PIN_CS GPIO2
#define LCD_PORT_CMD_DATA GPIOA
#define LCD_PIN_CMD_DATA GPIO5

#define CMD_SET_DISP_START_LINE  0x40
#define CMD_SET_PAGE  0xB0
#define CMD_SET_COLUMN_UPPER  0x10
#define CMD_SET_COLUMN_LOWER  0x00

#ifdef __cplusplus

class LCD {
    public:
        void init();
        void update();
        void powerOn();
        void dma_write();
        void dma_xfer_complete();
    private:
        volatile uint8_t _pixels[LCD_WIDTH * LCD_HEIGHT / 8];
        uint64_t _last_display_update = -LCD_UPDATE_MS;
        volatile bool _dma_active = false;
        int _count = 0;

        // Clear pixel buffer
        void clear();

        uint16_t px_offset_for_xy(uint16_t x, uint16_t y);
        // Render null-terminated text at x,y
        void draw_glyph(const uint8_t *glyph, uint8_t x, uint8_t y);
        void draw_text(const char *str, uint8_t x, uint8_t y);

        // Render bitmap of dimens w,h at x,y
        void draw_icon(const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t x, uint8_t y);

        // Push pixel buffer to LCD
        void render();

        // Set the backlight brightness
        void set_backlight(uint8_t brightness);
        void set_contrast(uint8_t val);

        void dma_init();
        void spi_init();

        // Pin management
        void cs_select();
        void cs_deselect();
        void reset_enable();
        void reset_disable();
        void mode_cmd();
        void mode_data();
        void write(uint8_t b);
};

extern LCD lcd;

#endif // __cplusplus

#endif // NEST_LCD_H
