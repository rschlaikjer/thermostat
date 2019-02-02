#include <nest/relay.hpp>

const uint32_t relay_port = GPIOA;
const uint16_t relay_pin = GPIO0;

static bool _relay_on = false;

void n_relay_init() {
  gpio_mode_setup(relay_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, relay_pin);
  n_relay_clear();
}

void n_relay_set() {
  gpio_set(relay_port, relay_pin);
  _relay_on = true;
}

void n_relay_clear() {
  gpio_clear(relay_port, relay_pin);
  _relay_on = false;
}

bool n_relay_is_on() { return _relay_on; }
