#ifndef NEST_RELAY_H
#define NEST_RELAY_H

#include <stdio.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

void n_relay_init(void);
void n_relay_set(void);
void n_relay_clear(void);
bool n_relay_is_on(void);

#endif // NEST_RELAY_H
