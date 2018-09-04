#ifndef NEST_UART_H
#define NEST_UART_H

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

const uint32_t NEST_UART = USART2;

extern "C" {
    void uart_setup(void);
    void uart_putc(char c);
    uint16_t uart_getc(void);
    void uart_write(const uint8_t *buf, size_t len);

    // Redirect printf to usart
    int _write(int file, char *ptr, int len);
}

#endif // NEST_UART_H
