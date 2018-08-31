#include "nest_sht.h"

uint8_t sht_read(const uint8_t sensor, double *temp_c, double *rh) {
    uint8_t cmd[2] = {0x2C, 0x06};
    uint8_t data[6];

    // Try and execute a write/read cycle
    if (!n_i2c_transfer(sensor, cmd, 2, data, 6)) {
        // If we failed, return failure here too
        return 0;
    }

    // If we succeeded, then data[0] and data[1] should be the raw temp value
    uint16_t sT = (data[0] << 8) | data[1];

    // And data[3, 4] should be the raw humidity data
    uint16_t sRH = (data[3] << 8) | data[4];

    // Convert the temperature reading to celcius
    *temp_c = -45.0 + 175 * (((double) sT) / ((double) (0xFFFF - 1)));

    // Conver the raw humidity reading to a percent
    *rh = 100.0 * (((double) sRH) / ((double) (0xFFFF - 1)));

    return 1;
}

void sht_log() {
    double temp, rh;

    uart_puts("Sensor 0: ");
    if (sht_read(SHT_0_ADDR, &temp, &rh)) {
        uart_puts("Temp: ");
        uart_putf(temp, 2);
        uart_puts(" RH: ");
        uart_putf(rh, 1);
        uart_putln("%");
    } else {
        uart_putln("READ ERROR");
    }

    uart_puts("Sensor 1: ");
    if (sht_read(SHT_1_ADDR, &temp, &rh)) {
        uart_puts("Temp: ");
        uart_putf(temp, 2);
        uart_puts(" RH: ");
        uart_putf(rh, 1);
        uart_putln("%");
    } else {
        uart_putln("READ ERROR");
    }
}
