#ifndef NEST_SPI_H
#define NEST_SPI_H

#include <stdio.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "nest_uart.h"

#define NEST_SPI SPI2
#define NEST_SPI_RCC RCC_SPI2

// Initialize the SPI bus
void n_spi_setup(void);

#endif // NEST_SPI_H
