#include "nest_uart.h"

void uart_setup() {
    // Enable clocks for UART
    rcc_periph_clock_enable(RCC_USART2);

    // Setup USART pins as alternate function
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);

    // Setup USART parameters
    usart_set_baudrate(NEST_UART, 115200);
    usart_set_databits(NEST_UART, 8);
    usart_set_parity(NEST_UART, USART_PARITY_NONE);
    usart_set_stopbits(NEST_UART, USART_CR2_STOPBITS_1);
    usart_set_mode(NEST_UART, USART_MODE_TX_RX);
    usart_set_flow_control(NEST_UART, USART_FLOWCONTROL_NONE);

    // Finally enable the USART
    usart_enable(NEST_UART);

    // One last thing: disable buffering on stdout.
    // No point optimizing for minimizing nonexistend syscalls...
    setbuf(stdout, NULL);

    printf("\r\nUART initialized\n");
}

int _write(int file, char *ptr, int len) {
    int i;

    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        for (i = 0; i < len; i++) {
            if (ptr[i] == '\n') {
                usart_send_blocking(NEST_UART, '\r');
            }
            usart_send_blocking(NEST_UART, ptr[i]);
        }
        return i;
    }
    errno = EIO;
    return -1;
}

void uart_putc(char c) {
    return usart_send_blocking(NEST_UART, c);
}

uint16_t uart_getc() {
    return usart_recv_blocking(NEST_UART);
}

void uart_write(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uart_putc(buf[i]);
    }
}
