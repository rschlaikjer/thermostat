#ifndef NEST_REALTIME_H
#define NEST_REALTIME_H

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

void systick_setup(void);
uint64_t millis(void);
void sleep(int milliseconds);

#endif // NEST_REALTIME_H
