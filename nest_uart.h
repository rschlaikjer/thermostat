#ifndef NEST_UART_H
#define NEST_UART_H

#include <math.h>
#include <stdint.h>
#include <unistd.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

const uint32_t NEST_UART = USART1;

void uart_setup(void);
void uart_puts(const char *string);
void uart_putln(const char *string);
void uart_putd(size_t i);
void uart_putd(ssize_t i);
void uart_putf(double n, uint8_t precision);
void uart_putx(size_t i);

#endif // NEST_UART_H
