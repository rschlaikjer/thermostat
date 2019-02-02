#include <nest/lcd.hpp>
#include <nest/font_8_13.hpp>

#define LCD_SPI SPI1

void LCD::init() {
    n_log("Initializing lcd... ");

    // PA5 = cmd/data selection
    // PA8 = reset
    // PB2 = CS
    gpio_mode_setup(LCD_PORT_RESET, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_PIN_RESET);
    gpio_mode_setup(LCD_PORT_CMD_DATA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_PIN_CMD_DATA);
    reset_enable();

    // CS
    gpio_mode_setup(LCD_PORT_CS, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_PIN_CS);
    cs_deselect();

    // TIM1 channel 2 is pwm output to LCD backlight
    rcc_periph_clock_enable(RCC_TIM1);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

    rcc_periph_clock_enable(RCC_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
    // Set output channel 2 to PWM mode 2 (inactive when counter < CR)
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM2);
    // Enable output channel 2
    timer_enable_oc_output(TIM1, TIM_OC2);
    // Active high
    timer_set_oc_polarity_high(TIM1, TIM_OC2N);
    // Enable auto-reload buffering
    timer_enable_preload(TIM1);
    timer_enable_oc_preload(TIM1, TIM_OC2);
    // Period in clock ticks
    timer_set_period(TIM1, 0xFF);
    timer_enable_break_main_output(TIM1);
    // Set output-compare value
    timer_set_oc_value(TIM1, TIM_OC2, 0xFF);
    // Update generation event
    timer_generate_event(TIM1, TIM_EGR_UG);
    // Enable
    timer_enable_counter(TIM1);

    spi_init();

    // Init DMA channel
    // dma_init();

    // Unit sense
    gpio_mode_setup(LCD_PORT_UNITSEL, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, LCD_PIN_UNITSEL);

    printf("done.\n");
}

bool LCD::use_celsius() {
    return gpio_get(LCD_PORT_UNITSEL, LCD_PIN_UNITSEL) == 0;
}

void LCD::spi_init() {
    // Enable clock for SPI and our GPIO bank
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_GPIOB);

    // Configure GPIOB, AF0
    // SCK = PB3
    // MISO = PB4
    // MOSI = PB5
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3 | GPIO4 | GPIO5);
    gpio_set_af(GPIOB, GPIO_AF0, GPIO3 | GPIO4 | GPIO5);

    // Reset SPI, SPI_CR1 register cleared, SPI is disabled
    spi_reset(LCD_SPI);

    // Set main SPI settings
    spi_init_master(
        LCD_SPI,
        SPI_CR1_BAUDRATE_FPCLK_DIV_8,
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1,
        SPI_CR1_MSBFIRST
    );

    /*
     * Set NSS management to software.
     *
     * Note:
     * Setting nss high is very important, even if we are controlling the GPIO
     * ourselves this bit needs to be at least set to 1, otherwise the spi
     * peripheral will not send any data out.
     */
    spi_enable_software_slave_management(LCD_SPI);
    spi_set_nss_high(LCD_SPI);

    // Data transfers are bidirectional
    spi_set_full_duplex_mode(LCD_SPI);
    spi_set_unidirectional_mode(LCD_SPI); // bidirectional but in 3-wire

    // Data is 8 bits
    spi_set_data_size(LCD_SPI, SPI_CR2_DS_8BIT);
    spi_fifo_reception_threshold_8bit(LCD_SPI);

    spi_enable(LCD_SPI);
}

void LCD::clear() {
    n_log("LCD::clear unimplemented\n");
}

void LCD::draw_text(const char *text, uint8_t x, uint8_t y) {
    uint8_t i = 0;
    while (*text) {
        unsigned char c = ((unsigned char) *text);
        if (font_ascii[c]) {
            draw_glyph(font_ascii[c], x + (8 * i), y);
        }
        i++;
        text++;
    }
}
void LCD::draw_glyph(const uint8_t *glyph, uint8_t x, uint8_t y) {
    // Input x, y are assuming 0,0 is top left corner when display is horizontal.
    // Pixels are actually addressed assuming 0,0 is top left when display is vertical
    // To convert;
    // pixel_n = x * buf_height + (buf_height - y)
    // Fonts are arranged as 8bit wide, 13bit high glyphs
    for (uint8_t rown = 0; rown < 13; rown++) {
        const uint8_t row = glyph[rown];
        for (uint8_t blit = 0; blit < 8; blit++) {
            // Convert that to a byte + bit
            const uint16_t py = y + rown;
            const uint16_t px = x + blit;
            const uint16_t byten = (py / 8) * 128 + px;
            // If we're trying to draw outside our buffer, skip
            if (byten >= sizeof(_pixels)) {
                continue;
            }
            const uint16_t bitn = py % 8;
            // Turn on that bit in our buffer
            if (row & (1 << (7 - blit))) {
              _pixels[byten] |= 1 << (7 - bitn);
            }
        }
    }
}

uint16_t LCD::px_offset_for_xy(uint16_t x, uint16_t y) {
    return (y / 8) * LCD_WIDTH + x;
}

void LCD::draw_icon(const uint8_t *bmp, uint8_t w, uint8_t h, uint8_t x, uint8_t y) {
    n_log("LCD::draw_icon unimplemented\n");
}

void LCD::render() {
    n_log("LCD::render unimplemented\n");
}

void LCD::powerOn() {
    // Reset
    cs_select();
    reset_enable();
    n_sleep(500);
    reset_disable();

    mode_cmd();

    /*
    write(CMD_SET_BIAS_7);
    write(CMD_SET_ADC_NORMAL);
    write(CMD_SET_COM_NORMAL);
    write(CMD_SET_DISP_START_LINE);

    write(CMD_SET_POWER_CONTROL | 0x4);
    n_sleep(50);
    write(CMD_SET_POWER_CONTROL | 0x6);
    n_sleep(50);
    write(CMD_SET_POWER_CONTROL | 0x7);
    n_sleep(10);

    write(CMD_SET_RESISTOR_RATIO | 0x6);

    write(CMD_SET_ALLPTS_ON);
    */

    write(0xa2); // LCD bias set at 1/9
    write(0xa0); // ADC select in normal mode
    write(0xc8); // Common output mode select: reverse direction (last 3 bits are ignored)
    write(0xc0); // COM output scan direction

    write(0x40); // Operating mode
    write(0x25); //  Resistor ratio

    set_contrast(0x10); // Set contrast, value experimentally determined, can set to 6-bit value, 0 to 63

    write(0x2f); // Power control set to operating mode: 7
    write(0xaf); // Display on

    write(0xa4); // All points off

    cs_deselect();
}

void LCD::set_contrast(uint8_t val) {
    mode_cmd();
    write(0x81);
    if (val > 63) {
        write(63);
    } else {
        write(val);
    }
}

void LCD::write(uint8_t b) {
    while (!(SPI_SR(LCD_SPI) & SPI_SR_TXE));
    SPI_DR8(LCD_SPI) = b;
    while (SPI_SR(LCD_SPI) & SPI_SR_BSY);
}

void LCD::update() {
    // If not time to update, return.
    if (millis() - _last_display_update < LCD_UPDATE_MS) {
        return;
    }

    // Bump the update ts
    _last_display_update = millis();

    // Clear our buffer
    memset(((void *)_pixels), 0x00, (LCD_WIDTH * LCD_HEIGHT) / 8);

    // Get the current unit
    const bool is_celsius = use_celsius();
    const char unit = is_celsius ? 'C' : 'F';

    // Small buffer for formatting
    char buf[32];

    // Draw the current temperature
    if (Sensors.has_temp()) {
        float temp = Sensors.get_temp();
        if (!is_celsius)
            temp = (temp * (9.0/5.0)) + 32.0;
        sprintf(buf, "%2.1f%c", temp, unit);
    } else {
        sprintf(buf, "--.-%c", unit);
    }
    draw_text(buf, 8, 0);

    // Target temp
    float target_temp = 18;
    if (!is_celsius)
        target_temp = (target_temp * (9.0/5.0)) + 32.0;
    sprintf(buf, "(%2.1f%c)", target_temp, unit);
    draw_text(buf, 128 - (strlen(buf) + 1) * 8, 0);

    // RH + Lux
    if (Sensors.has_rh()) {
        sprintf(buf, "%2.0f%% RH %2.0f%% LUX", Sensors.get_rh(), Sensors.get_brightness());
    } else {
        sprintf(buf, "--%% RH %2.0f%% LUX", Sensors.get_brightness());
    }
    draw_text(buf, 8, 14);


    // Date & time
    const time_t now = n_est() / 1000;
    struct tm local = *localtime(&now);
    strftime(buf, 21, "%H:%M %d/%m/%Y", &local);
    draw_text(buf, 0, 53);

    // Home the data cursor to the origin
    cs_select();
    const uint16_t page_size = 128;
    const uint8_t page_count = 8;
    for (uint16_t page = 0; page < page_count; page++) {
        mode_cmd();
        write(CMD_SET_PAGE | (7 - page));
        write(0x00);
        write(0x10);
        write(CMD_SET_DISP_START_LINE);
        mode_data();
        for (uint16_t i = page * page_size; i < (page + 1) * page_size; i++) {
            write(_pixels[i]);
        }
    }
    cs_deselect();

    // Start a DMA transfer to fill in the pixel data
    //dma_write();
}

void LCD::dma_init() {
    // Enable DMA clock
    rcc_periph_clock_enable(RCC_DMA1);
    // Dma controller 1 channel 3, connected to SPI1
    dma_channel_reset(DMA1, DMA_CHANNEL3);
    // SPI1 data register as output
    dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&SPI1_DR);
    // Use our pixel buffer as the source data
    dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)_pixels);
    dma_set_number_of_data(DMA1, DMA_CHANNEL3, (LCD_WIDTH * LCD_HEIGHT) / 8);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
    // Need to increment memory address
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
    // Don't need to increment peripheral address, since the SPI DR doesn't move
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL3);
    // 8 bit transfers
    dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
    // High priority
    dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);
    // Use interrupts to detect transfer complete
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);
    // Enable interrupts in NVIC
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);
    // Enable
    dma_enable_channel(DMA1, DMA_CHANNEL3);
}

void LCD::dma_write() {
    // Don't allow double-starting the transfer
    if (_dma_active) {
        return;
    }

    // Set the source & number of data
    dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)_pixels);
    dma_set_number_of_data(DMA1, DMA_CHANNEL3, (LCD_WIDTH * LCD_HEIGHT) / 8);
    dma_enable_channel(DMA1, DMA_CHANNEL3);

    // Select the LCD
    cs_select();
    // Data mode
    mode_data();
    // Mark DMA as happening
    _dma_active = true;
    // Begin
    spi_enable_tx_dma(SPI1);
}

void LCD::dma_xfer_complete() {
    // Wait to ensure transfer is _really_ complete
    while (SPI_SR(SPI1) & SPI_SR_BSY);
    // Deselect chip
    cs_deselect();
    // Turn off the DMA channel
    spi_disable_tx_dma(SPI1);
    dma_disable_channel(DMA1, DMA_CHANNEL3);
    // Un-mark the dma as active
    _dma_active = false;
}

void LCD::set_backlight(uint8_t brightness) {
    timer_set_oc_value(TIM1, TIM_OC2, 255 - brightness);
}

void LCD::cs_select() {
    gpio_clear(LCD_PORT_CS, LCD_PIN_CS);
}
void LCD::cs_deselect() {
    gpio_set(LCD_PORT_CS, LCD_PIN_CS);
}

void LCD::reset_enable() {
    gpio_clear(LCD_PORT_RESET, LCD_PIN_RESET);
}
void LCD::reset_disable() {
    gpio_set(LCD_PORT_RESET, LCD_PIN_RESET);
}

void LCD::mode_cmd() {
    gpio_clear(LCD_PORT_CMD_DATA, LCD_PIN_CMD_DATA);
}

void LCD::mode_data() {
    gpio_set(LCD_PORT_CMD_DATA, LCD_PIN_CMD_DATA);
}
