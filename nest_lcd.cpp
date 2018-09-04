#include "nest_lcd.h"

static void i2c_setup(void) {
    // Ensure clock is enabled
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_set_i2c_clock_hsi(I2C1);

    // Reset any existing i2c conf
    i2c_reset(I2C1);

    // Configure GPIOs
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);

    i2c_peripheral_disable(I2C1);

    i2c_enable_analog_filter(I2C1);
    i2c_set_digital_filter(I2C1, 0);

    i2c_set_speed(I2C1, i2c_speed_sm_100k, 48);

    i2c_set_7bit_addr_mode(I2C1);
    i2c_peripheral_enable(I2C1);
}

#define ADDR 0x60

int16_t readInt(uint8_t address, uint8_t reg) {
    uint8_t data[2];
    i2c_transfer7(I2C1, address, &reg, 1, data, 2);
    return (((int16_t) data[0]) << 8) | data[1];
}

uint16_t readUInt(uint8_t address, uint8_t reg) {
    uint8_t data[2];
    i2c_transfer7(I2C1, address, &reg, 1, data, 2);
    return (((uint16_t) data[0]) << 8) | data[1];
}

void lcd_init() {
    uint8_t init_commands[11];
    init_commands[0] = 0x3A; // 8 bit data length extension Bit RE=1; REV=0
    init_commands[1] = 0x09; // 4 line display
    init_commands[2] = 0x06; // Bottom view
    init_commands[3] = 0x1E; // BS1 = 1
    init_commands[4] = 0x39; // 8 bit data length extension Bit RE=0; IS=1
    init_commands[5] = 0x1B; // BS0=1 -> Bias=1/6
    init_commands[6] = 0x6E; // Divider on, set value
    init_commands[7] = 0x57; // Booster on and set contrast (DB1=C5, DB0=C4)
    init_commands[8] = 0x72; // Set contrast (DB3-DB0=C3-C0)
    init_commands[9] = 0x38; // 8 bit data length extension Bit RE=0; IS=0
    init_commands[10] = 0x0F; // Display on, cursor on, blink on

    i2c_transfer7(NEST_I2C, LCD_I2C_ADDR, init_commands, 11, NULL, 0);
}

/*
 * Switch ROM used for LCD.
 * Rom must be between 1 and 3.
 */
void lcd_rom_select(uint8_t rom) {
    uint8_t rom_cmd[4];
    rom_cmd[0] = 0x3A; // RE = 1
    rom_cmd[1] = 0x72; // Rom select command
    switch (rom) {
        case 3:
            rom_cmd[2] = 0x0C;
            break;
        case 2:
            rom_cmd[2] = 0x04;
            break;
        case 1:
        default:
            rom_cmd[2] = 0x00;
    }
    rom_cmd[3] = 0x38; // RE = 0
    i2c_transfer7(NEST_I2C, LCD_I2C_ADDR, rom_cmd, 4, NULL, 0);
}
