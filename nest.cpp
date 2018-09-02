#include "nest.h"
#include <stdio.h>
#include <errno.h>


extern "C" {
    #include "winc1500/socket/include/socket.h"
    #include "winc1500/driver/include/m2m_periph.h"
    #include "winc1500/driver/include/m2m_wifi.h"
    int _write(int file, char *ptr, int len);
}

static void clock_setup(void) {
    // Set clock at 48MHz from internal oscillator
    rcc_clock_setup_in_hsi_out_48mhz();

    // Enable GPIO clocks
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // Enable clock for SPI
    rcc_periph_clock_enable(RCC_SPI1);
}


static void gpio_setup(void) {
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO9 | GPIO10);
}

#define ADDR 0x60

int _write(int file, char *ptr, int len)
{
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

int main(void) {
    nest_init();
    while (true) {
        nest_event_loop();
    }
    return 0;
}

static void wifi_cb(uint8_t u8MsgType, void *pvMsg) {
    uart_putln("Got wifi callback!");
}

uint8_t wifi_init() {
    uart_putln("BSP init");
    nm_bsp_init();
    tstrWifiInitParam param;
    param.pfAppWifiCb = wifi_cb;
    uart_putln("m2m wifi init");
    int8_t ret = m2m_wifi_init(&param);
    if (ret == M2M_SUCCESS) {
        uart_putln("m2m wifi init success!");
    } else {
        if (ret == M2M_ERR_FW_VER_MISMATCH) {
            uart_putln("m2m wifi init FW mismatch!");
        } else if (ret == M2M_ERR_INVALID) {
            uart_putln("m2m wifi init invalid!");

        } else {
            uart_puts("m2m wifi init failed:");
            uart_putx(ret);
            uart_putln("");
        }
    }

#ifdef CONF_PERIPH
    m2m_periph_gpio_set_val(M2M_PERIPH_GPIO4, 0);
    m2m_periph_gpio_set_val(M2M_PERIPH_GPIO5, 0);
    m2m_periph_gpio_set_val(M2M_PERIPH_GPIO6, 0);
    m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO4, 1);
    m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO5, 1);
    m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO6, 1);
#endif


    return ret;
}

void nest_init() {
    clock_setup();
    systick_setup();
    gpio_setup();
    uart_setup();
    n_i2c_setup();
    n_spi_setup();
    adc_setup();

    wifi_init();
    m2m_wifi_handle_events(NULL);

    // tstrM2mRev rev;
    // nm_get_firmware_info(&rev);
    // uart_puts("Major: ");
    // uart_putd(rev.u8FirmwareMajor);
    // uart_puts(" Minor: ");
    // uart_putd(rev.u8FirmwareMinor);
    // uart_puts(" Patch: ");
    // uart_putd(rev.u8FirmwarePatch);
    // uart_putln("");
}

#define READ_0_RATE_MS 3000
#define READ_1_RATE_MS 30000

uint64_t last_read_0 = 0;
uint64_t last_read_1 = 0;

void nest_event_loop() {
    m2m_wifi_handle_events(NULL);
}

void nest_event_loop_old() {
    double temp, rh;
    if (millis() - last_read_0 > READ_0_RATE_MS) {
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
        last_read_0 = millis();
    }
    if (millis() - last_read_1 > READ_1_RATE_MS) {
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
        last_read_1 = millis();
    }

}
