#include "nest_lcd.h"

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
    printf("Initializing lcd... ");

    uint8_t init_commands[] = {
        0x80, 0x3A,  // 8 bit data length extension Bit RE=1; REV=0
        0x80, 0x09,  // 4 line display
        0x80, 0x06,  // Bottom view
        0x80, 0x1A,  // BS1 = 1
        0x80, 0x72,  // ROM select
        0xC0, 0x00,  // Rom 1
        0x80, 0x39,  // 8 bit data length extension Bit RE=0; IS=1
        0x80, 0x1B,  // BS0=1 -> Bias=1/6
        0x80, 0x6E,  // Divider on, set value
        0x80, 0x57,  // Booster on and set contrast (DB1=C5, DB0=C4)
        0x80, 0x72,  // Set contrast (DB3-DB0=C3-C0)
        0x80, 0x38,  // 8 bit data length extension Bit RE=0; IS=0
        0x80, 0x0C,  // Display on, no cursor, no blink
        // Clear continuation bit
        0x00, 0x01  // Clear display, cursor return
    };

    if (NEST_I2C_XFER_OK !=
            n_i2c_transfer(LCD_I2C_ADDR, init_commands, sizeof(init_commands), NULL, 0)) {
        printf("failed!\n");
        return;
    }

    printf("done.\n");
}

/*
 * Switch ROM used for LCD.
 * Rom must be between 1 and 3.
 */
void lcd_rom_select(uint8_t rom) {
    // Convert rom to cmd
    if (rom == 2) {
        rom = 0x0C;
    } else if (rom == 1) {
        rom = 0x07;
    } else {
        rom = 0x0;
    }

    uint8_t rom_cmd[] = {
        0x80, 0x3A, // RE = 1
        0x80, 0x72, // Rom select
        0xC0, rom, // ROM Id
        0x00, 0x38, // RE = 0
    };
    if (NEST_I2C_XFER_OK != n_i2c_transfer(LCD_I2C_ADDR, rom_cmd, 4, NULL, 0)) {
        printf("Failed to change lcd rom\n");
    }
}

void lcd_clear() {
    uint8_t cmd[2];
    cmd[0] = 0x00; // Command, no continuation
    cmd[1] = 0x01; // Clear display, cursor return
    if (NEST_I2C_XFER_OK != n_i2c_transfer(LCD_I2C_ADDR, cmd, sizeof(cmd), NULL, 0)) {
        printf("Failed to clear lcd\n");
    }
}

uint8_t lcd_home() {
    uint8_t cmd[2];
    cmd[0] = 0x00; // Command, no continuation
    cmd[1] = 0x02; // Cursor to 0,0
    if (NEST_I2C_XFER_OK != n_i2c_transfer(LCD_I2C_ADDR, cmd, sizeof(cmd), NULL, 0)) {
        printf("Failed to home cursor\n");
        return 1;
    }
    return 0;
}

uint8_t lcd_set_line(uint8_t line) {
    // 0 <= line <= 3
    line &= 0x3;
    uint8_t cmd[2];
    cmd[0] = 0x00; // Command, no continuation
    cmd[1] = 0x80 + (line << 5);
    if (NEST_I2C_XFER_OK != n_i2c_transfer(LCD_I2C_ADDR, cmd, sizeof(cmd), NULL, 0)) {
        printf("Failed to set line\n");
        return 1;
    }
    return 0;
}

static uint64_t last_display_update = -LCD_UPDATE_MS;
void lcd_update() {
    if (millis() - last_display_update < LCD_UPDATE_MS) {
        return;
    }
    last_display_update = millis();

    uint8_t data[22] = {};
    data [0] = 0x40;

    // Line 1
    const uint8_t celsius = 0;
    const uint8_t relay_on = 1;
    uint8_t len;
    double temp, rh;
    sht_read(SHT_0_ADDR, &temp, &rh);
    if (lcd_set_line(0)) { return; }
    memset(&data[1], 0x20, 20);
    if (celsius) {
        len = snprintf(reinterpret_cast<char *>(&data[1]), 21,
            "   %2.1fC  (%2.1fC)",
            temp, 22.0
        );
        data[len+1] = ' ';
    } else {
        len = snprintf(reinterpret_cast<char *>(&data[1]), 21,
            "    %3.0fF  (%.0fF)",
            71.65, 65.0
        );
        data[len+1] = ' ';
    }
    if (relay_on) {
        data[1] = 0x12;
        data[20] = 0x12;
    }
    if (NEST_I2C_XFER_OK != n_i2c_transfer(LCD_I2C_ADDR, data, sizeof(data)-1, NULL, 0)) {
        printf("LCD update failed\n");
        return;
    }

    // Line 2
    if (lcd_set_line(1)) { return; }
    memset(&data[1], 0x20, 20);
    uint16_t brightness = adc_read();
    double brightness_perc = (brightness * 100) / 4096.0;
    len = snprintf(reinterpret_cast<char *>(&data[1]), 21,
        " Rh: %2.0f%%  Lux: %2.0f%%",
        rh, brightness_perc
    );
    data[len+1] = ' ';
    if (NEST_I2C_XFER_OK != n_i2c_transfer(LCD_I2C_ADDR, data, sizeof(data)-1, NULL, 0)) {
        printf("LCD update failed\n");
        return;
    }

    // Line 3
    if (lcd_set_line(2)) { return; }
    memset(&data[1], 0x20, 20);
    const time_t now = n_est() / 1000;
    struct tm local = *localtime(&now);
    len = strftime(reinterpret_cast<char *>(&data[1]), 21,
        "%Y-%m-%d  %H:%M:%S", &local);
    data[len+1] = ' ';
    if (NEST_I2C_XFER_OK != n_i2c_transfer(LCD_I2C_ADDR, data, sizeof(data)-1, NULL, 0)) {
        printf("LCD update failed\n");
        return;
    }

    // Line 4
    if (lcd_set_line(3)) { return; }
    memset(&data[1], 0x20, 20);
    len = snprintf(reinterpret_cast<char *>(&data[1]), 21,
        "Up: %6lld   Wifi OK", millis() / 1000
    );
    data[len+1] = ' ';
    if (NEST_I2C_XFER_OK != n_i2c_transfer(LCD_I2C_ADDR, data, sizeof(data)-1, NULL, 0)) {
        printf("LCD update failed\n");
        return;
    }
}
