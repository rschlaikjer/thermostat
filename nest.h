#ifndef NEST__H
#define NEST_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "nest_uart.h"
#include "nest_i2c.h"
#include "nest_lcd.h"
#include "nest_sht.h"
#include "nest_realtime.h"

void nest_init(void);

void nest_event_loop(void);

#endif // NEST_H
