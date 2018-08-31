#ifndef NEST_SHT_H
#define NEST_SHT_H

#include <libopencm3/stm32/i2c.h>

#include "nest_i2c.h"
#include "nest_uart.h"

#define SHT_0_ADDR 0x44
#define SHT_1_ADDR 0x45

uint8_t sht_read(const uint8_t sensor, double *temp_c, double *rh);

void sht_log(void);

#endif // NEST_SHT_H
