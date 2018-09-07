#ifndef NEST_INTERRUPTS_H
#define NEST_INTERRUPTS_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

#include "winc1500/bsp/include/nm_bsp_opencm3.h"
#include "nest_uart.h"

void interrupt_init(void);

void interrupt_init_buttons();
void interrupt_init_winc();

#endif // NEST_INTERRUPTS_H
