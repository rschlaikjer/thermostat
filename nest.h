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
#include "nest_adc.h"
#include "nest_spi.h"
#include "nest_wifi.h"
#include "nest_secrets.h"

const uint32_t *STM32F0_CHIP_ID = reinterpret_cast<uint32_t *>(0x1FFFF7AC);

void nest_init(void);

void nest_event_loop(void);

#endif // NEST_H
