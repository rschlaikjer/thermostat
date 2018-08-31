#include "nest_realtime.h"

static volatile uint64_t _millis = 0;

void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
    STK_CVR = 0;
    // Interrupt every 1ms
    systick_set_reload(rcc_ahb_frequency / 8 / 1000);
    systick_interrupt_enable();
    systick_counter_enable();
}

void sys_tick_handler(void) {
    _millis++;
}

uint64_t millis(void) {
    return _millis;
}

void sleep(int milliseconds) {
    const uint32_t until = millis() + milliseconds;
    while (millis() < until);
}

