#ifndef NEST_INTERRUPTS_H
#define NEST_INTERRUPTS_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

#include <bsp/include/nm_bsp_opencm3.h>

#include <nest/uart.hpp>

#ifdef __cplusplus
#include <nest/lcd.hpp>
#endif

void interrupt_init(void);

void interrupt_init_buttons();
void interrupt_init_winc();

#endif // NEST_INTERRUPTS_H
