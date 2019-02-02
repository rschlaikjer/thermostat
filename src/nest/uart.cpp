#include <nest/uart.hpp>

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

  printf("\r\n");
  for (uint8_t i = 0; i < 80; i++)
    uart_putc('=');
  printf("\r\n");
  n_log("UART initialized\n");
}

void _putchar(char c) {
  if (c == '\n')
    uart_putc('\r');
  uart_putc(c);
}

int _write(int file, char *ptr, int len) {
  int i;

  if (file == STDOUT_FILENO || file == STDERR_FILENO) {
    // Log the msg to uart
    for (i = 0; i < len; i++) {
      if (ptr[i] == '\n') {
        usart_send_blocking(NEST_UART, '\r');
      }
      usart_send_blocking(NEST_UART, ptr[i]);
    }

    // Also try and send it over the air
    wifi_fsm.send_log_msg(ptr, len);

    return i;
  }
  errno = EIO;
  return -1;
}

void uart_putc(char c) { return usart_send_blocking(NEST_UART, c); }

uint16_t uart_getc() { return usart_recv_blocking(NEST_UART); }

uint16_t uart_poll() {
  if ((USART_ISR(NEST_UART) & USART_ISR_RXNE) == 0) {
    return -1;
  }
  return USART_RDR(NEST_UART) & USART_RDR_MASK;
}

void uart_write(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uart_putc(buf[i]);
  }
}

int n_log(const char *format, ...) {
  // Print the time
  const time_t now = n_utc() / 1000;
  char buf[20];
  // struct tm local = *localtime(&now);
  // strftime(buf, 21, "%Y-%m-%d  %H:%M:%S", &local);
  printf("[%s] ", buf);

  // Proxy args to actual printf
  va_list args;
  va_start(args, format);
  int b = vprintf(format, args);
  va_end(args);

  return b;
}
