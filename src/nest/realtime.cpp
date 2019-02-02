#include <nest/realtime.hpp>

namespace {
volatile uint64_t _millis = 0;
}

static uint64_t _utc_offset = 0;

void systick_setup(void) {
  systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
  STK_CVR = 0;
  // Interrupt every 1ms
  systick_set_reload(rcc_ahb_frequency / 8 / 1000);
  systick_interrupt_enable();
  systick_counter_enable();
}

void sys_tick_handler(void) { _millis++; }

uint64_t millis(void) { return _millis; }

void n_sleep(uint64_t milliseconds) {
  const uint64_t until = millis() + milliseconds;
  while (millis() < until)
    ;
}

uint64_t n_utc(void) { return _utc_offset + millis(); }

uint64_t n_est(void) { return n_utc() - EST_OFFSET; }

void set_utc_offset(uint64_t current_utc) {
  _utc_offset = current_utc - millis();
}
