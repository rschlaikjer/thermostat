#ifndef NEST_REALTIME_H
#define NEST_REALTIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

void systick_setup(void);
uint64_t millis(void);
void n_sleep(uint64_t milliseconds);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // NEST_REALTIME_H
