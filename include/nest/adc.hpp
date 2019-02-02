#ifndef NEST_ADC_H
#define NEST_ADC_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#include <nest/uart.hpp>

#define NEST_ADC ADC1

void adc_setup(void);
uint16_t adc_read(void);

#endif // NEST_ADC_H
