#ifndef NEST__H
#define NEST_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <nest/adc.hpp>
#include <nest/i2c.hpp>
#include <nest/interrupts.hpp>
#include <nest/lcd.hpp>
#include <nest/realtime.hpp>
#include <nest/relay.hpp>
#include <nest/secrets.hpp>
#include <nest/sensors.hpp>
#include <nest/sht.hpp>
#include <nest/spi.hpp>
#include <nest/uart.hpp>
#include <nest/wifi.hpp>
#include <nest/wifi_firmware.hpp>

const uint32_t *STM32F0_CHIP_ID = reinterpret_cast<uint32_t *>(0x1FFFF7AC);

const uint32_t WATCHDOG_TIMEOUT_MS = 10000;

void nest_init(void);

void nest_event_loop(void);

#endif // NEST_H
