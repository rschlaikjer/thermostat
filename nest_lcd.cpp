#include "nest_lcd.h"

void LCD::init() {
    n_log("Initializing lcd... ");

    // TIM1 channel 2 is pwm output to LCD backlight
    rcc_periph_clock_enable(RCC_TIM1);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
    // Set output channel 2 to PWM mode 2 (inactive when counter < CR)
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM2);
    // Enable output channel 2
    timer_enable_oc_output(TIM1, TIM_OC2);
    // Active high
    timer_set_oc_polarity_high(TIM1, TIM_OC2N);
    // Enable auto-reload buffering
    timer_enable_preload(TIM1);
    timer_enable_oc_preload(TIM1, TIM_OC2);
    // Period in clock ticks
    timer_set_period(TIM1, 0xFF);
    // Set output-compare value
    timer_set_oc_value(TIM1, TIM_OC2, 0xFF);
    // Update generation event
    timer_generate_event(TIM1, TIM_EGR_UG);
    // Enable
    timer_enable_counter(TIM1);
    printf("done.\n");
}

void LCD::clear() {
    n_log("LCD::clear unimplemented\n");
}

void LCD::draw_text(const char *text, uint8_t x, uint8_t y) {
    n_log("LCD::draw_text unimplemented\n");
}

void LCD::draw_icon(const uint8_t *bmp, uint8_t w, uint8_t h, uint8_t x, uint8_t y) {
    n_log("LCD::draw_icon unimplemented\n");
}

void LCD::render() {
    n_log("LCD::render unimplemented\n");
}

void LCD::update() {
    // If not time to update, return.
    if (millis() - _last_display_update < LCD_UPDATE_MS) {
        return;
    }

    // Bump the update ts
    _last_display_update = millis();
}

void LCD::set_backlight(uint8_t brightness) {
    timer_set_oc_value(TIM1, TIM_OC2, brightness);
}
