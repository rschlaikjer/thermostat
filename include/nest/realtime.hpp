#ifndef NEST_REALTIME_H
#define NEST_REALTIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

#define EDT_OFFSET 14400000
#define EST_OFFSET 18000000

void systick_setup(void);
uint64_t millis(void);
uint64_t n_utc(void);
uint64_t n_est(void);
void set_utc_offset(uint64_t current_utc);
void n_sleep(uint64_t milliseconds);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // NEST_REALTIME_H
