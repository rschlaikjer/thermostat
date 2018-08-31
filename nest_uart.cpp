#include "nest_uart.h"

void uart_setup() {
    // Enable clocks for UART
    rcc_periph_clock_enable(RCC_USART2);

    // Setup TX pin as alternate function
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);

    // Setup USART parameters
    usart_set_baudrate(NEST_UART, 9600);
    usart_set_databits(NEST_UART, 8);
    usart_set_parity(NEST_UART, USART_PARITY_NONE);
    usart_set_stopbits(NEST_UART, USART_CR2_STOPBITS_1);
    usart_set_mode(NEST_UART, USART_MODE_TX);
    usart_set_flow_control(NEST_UART, USART_FLOWCONTROL_NONE);

    // Finally enable the USART
    usart_enable(NEST_UART);

    uart_putln("\r\nUART initialized");
}

void uart_putc(char c) {
    usart_send_blocking(NEST_UART, c);
}

void uart_puts(const char *string) {
    while (*string) {
        uart_putc(string[0]);
        string++;
    }
}

void uart_putln(const char *string) {
    uart_puts(string);
    uart_puts("\r\n");
}

char hexchr(uint8_t c) {
    if (c < 10) {
        return '0' + c;
    }
    return 'A' + (c - 10);
}

void uart_putd(ssize_t n) {
    char buf[22] = {'0'};
    uint8_t i = 0;
    const bool negative = n < 0;

    // Convert to number in buffer
    do {
        buf[i++] = n % 10 + '0';
    } while ((n /= 10) > 0);

    // If negative, print -
    if (negative)
        uart_putc('-');

    // Write out buffer in reverse order
    do {
        uart_putc(buf[--i]);
    } while (i > 0);
}

void uart_putd(size_t n) {
    char buf[22] = {'0'};
    uint8_t i = 0;

    // Convert to number in buffer
    do {
        buf[i++] = n % 10 + '0';
    } while ((n /= 10) > 0);

    // Convert to number in buffer
    do {
        uart_putc(buf[--i]);
    } while (i > 0);
}

void uart_putx(size_t n) {
    char buf[22] = {'0'};
    uint8_t i = 0;

    // Convert to number in buffer
    do {
        const uint8_t v = n % 16;
        buf[i++] = v > 9 ? 'A' + v - 9 : v + '0';
    } while ((n /= 16) > 0);

    // Convert to number in buffer
    do {
        uart_putc(buf[--i]);
    } while (i > 0);
}

double fpow(double a, int b) {
    double acc = a;
    b--;
    while (b--) {
        acc *= a;
    }
    return acc;
}

void uart_putf(double n, uint8_t precision) {
    // Extract integer part
    const ssize_t ipart = (ssize_t) n;
    uart_putd(ipart);

    // Extract floating part
    double fpart = n - (double) ipart;
    uart_putc('.');
    fpart = fpart * fpow(10, precision);
    uart_putd((size_t) fpart);
}
