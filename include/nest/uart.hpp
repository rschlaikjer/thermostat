#ifndef NEST_UART_H
#define NEST_UART_H

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <nest/realtime.hpp>

#include <printf/printf.h>

#ifdef __cplusplus
#include <nest/wifi.hpp>
#endif

#define NEST_UART USART2

#ifdef __cplusplus
extern "C" {
#endif
    void uart_setup(void);
    void uart_putc(char c);
    uint16_t uart_getc(void);
    uint16_t uart_poll(void);
    void uart_write(const uint8_t *buf, size_t len);

    // Redirect printf to usart
    int _write(int file, char *ptr, int len);

    // Log method
    int n_log(const char *format, ...);
#ifdef __cplusplus
}
#endif

#endif // NEST_UART_H
