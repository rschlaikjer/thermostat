#include <nest/interrupts.hpp>

void exti4_15_isr() {
  // Temp minus
  if (exti_get_flag_status(EXTI6)) {
    exti_reset_request(EXTI6);
    // n_log("Btn up\n");
  }

  // Temp plus
  if (exti_get_flag_status(EXTI7)) {
    exti_reset_request(EXTI7);
    // n_log("Btn down\n");
  }

  // WiFi
  if (exti_get_flag_status(EXTI10)) {
    exti_reset_request(EXTI10);
    winc_interrupt_bridge();
  }
}

// LCD transfer DMA interrupt
void dma1_channel2_3_isr(void) {
  // Clear transfer complete interrupt flag
  if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF)) {
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_TCIF);
    lcd.dma_xfer_complete();
  }
}

void interrupt_init() {
  n_log("Initializing interrupts... ");

  // Configure triggers
  interrupt_init_buttons();
  interrupt_init_winc();

  // Enable the interrupt in the NVIC
  nvic_set_priority(NVIC_EXTI4_15_IRQ, 1);
  nvic_enable_irq(NVIC_EXTI4_15_IRQ);

  printf("done.\n");
}

void interrupt_init_buttons() {
  // Configure button pins as input, pulldown
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO6);
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO7);

  // Enable triggers for buttons
  exti_set_trigger(EXTI6, EXTI_TRIGGER_RISING);
  exti_set_trigger(EXTI7, EXTI_TRIGGER_RISING);

  // Configure source port
  exti_select_source(EXTI6, GPIOA);
  exti_select_source(EXTI7, GPIOA);

  // Enable exti interrupts & events
  exti_enable_request(EXTI6);
  exti_enable_request(EXTI7);
}

void interrupt_init_winc() {
  // Trigger on rising edge of EXTI 10
  exti_set_trigger(EXTI10, EXTI_TRIGGER_FALLING);

  // Configure EXTI10 to use GPIOB
  exti_select_source(EXTI10, GPIOB);

  // Enable exti interrupts & events
  exti_enable_request(EXTI10);
}
