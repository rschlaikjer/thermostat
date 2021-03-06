#include <nest/spi.hpp>

void n_spi_setup(void) {
  n_log("Initializing spi... ");

  // Enable clock for SPI and our GPIO bank
  rcc_periph_clock_enable(NEST_SPI_RCC);
  rcc_periph_clock_enable(RCC_GPIOB);

  // Configure GPIOB, AF0
  // NSS = PB12
  // SCK = PB13
  // MISO = PB14
  // MOSI = PB15
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO12 | GPIO13 | GPIO14 | GPIO15);
  // gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
  gpio_set_af(GPIOB, GPIO_AF0, GPIO13 | GPIO14 | GPIO15);

  // Reset SPI, SPI_CR1 register cleared, SPI is disabled
  spi_reset(NEST_SPI);

  // Set main SPI settings
  spi_init_master(NEST_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_8,
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(NEST_SPI);
  spi_set_nss_high(NEST_SPI);

  // Data transfers are bidirectional
  spi_set_full_duplex_mode(NEST_SPI);
  spi_set_unidirectional_mode(NEST_SPI); // bidirectional but in 3-wire

  // Data is 8 bits
  spi_set_data_size(NEST_SPI, SPI_CR2_DS_8BIT);
  spi_fifo_reception_threshold_8bit(NEST_SPI);

  // Enable 8 bit CRC
  // spi_set_crcl_8bit(NEST_SPI);
  // spi_enable_crc(NEST_SPI);

  /* Enable NEST_SPI periph. */
  spi_enable(NEST_SPI);

  printf("done.\n");
}
