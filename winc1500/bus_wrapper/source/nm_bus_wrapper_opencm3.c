/*
 * Variants may define an alternative SPI instace to use for WiFi101.
 * If not defined the following defaults are used:
 *   WINC1501_SPI    - SPI
 */

#include "bus_wrapper/include/nm_bus_wrapper_opencm3.h"

tstrNmBusCapabilities egstrNmBusCapabilities = {
    NM_BUS_MAX_TRX_SZ
};

uint8_t spi_xfer8(uint8_t write) {
    // Write the data
    while (!(SPI_SR(NEST_SPI) & SPI_SR_TXE));
    SPI_DR8(NEST_SPI) = write;

    // Read a response
    while (!(SPI_SR(NEST_SPI) & SPI_SR_RXNE));
    uint8_t r =  SPI_DR8(NEST_SPI);

    return r;
}

static int8_t spi_rw(uint8_t* pu8Mosi, uint8_t* pu8Miso, uint16_t u16Sz) {
    uint8_t u8Dummy = 0;
    uint8_t u8SkipMosi = 0, u8SkipMiso = 0;

    if (!pu8Mosi) {
        pu8Mosi = &u8Dummy;
        u8SkipMosi = 1;
    }
    else if(!pu8Miso) {
        pu8Miso = &u8Dummy;
        u8SkipMiso = 1;
    }
    else {
        return M2M_ERR_BUS_FAIL;
    }

    gpio_clear(gWincCSPort, gWincCSPin);

    while (u16Sz) {
        *pu8Miso = spi_xfer8(*pu8Mosi);

        u16Sz--;
        if (!u8SkipMiso)
            pu8Miso++;
        if (!u8SkipMosi)
            pu8Mosi++;
    }

    gpio_set(gWincCSPort, gWincCSPin);

    return M2M_SUCCESS;
}

/*
*   @fn     nm_bus_init
*   @brief  Initialize the bus wrapper
*   @return M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*   @author M.S.M
*   @date   28 oct 2013
*   @version    1.0
*/
int8_t nm_bus_init(void *pvInitValue) {
    int8_t result = M2M_SUCCESS;

    /* Configure CS PIN. */
    // pinMode(gi8Winc1501CsPin, OUTPUT);
    // digitalWrite(gi8Winc1501CsPin, HIGH);

    /* Reset WINC1500. */
    nm_bsp_reset();
    nm_bsp_sleep(1);

    return result;
}

/*
*   @fn     nm_bus_ioctl
*   @brief  send/receive from the bus
*   @param[IN]  u8Cmd
*                   IOCTL command for the operation
*   @param[IN]  pvParameter
*                   Arbitrary parameter depenging on IOCTL
*   @return M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*   @author M.S.M
*   @date   28 oct 2013
*   @note   For SPI only, it's important to be able to send/receive at the same time
*   @version    1.0
*/
int8_t nm_bus_ioctl(uint8_t u8Cmd, void* pvParameter) {
    int8_t s8Ret = 0;
    switch(u8Cmd)
    {
        case NM_BUS_IOCTL_RW: {
            tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
            s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
        }
        break;
        default:
            s8Ret = -1;
            M2M_ERR("invalide ioclt cmd\n");
            break;
    }

    return s8Ret;
}

/*
*   @fn     nm_bus_deinit
*   @brief  De-initialize the bus wrapper
*   @author M.S.M
*   @date   28 oct 2013
*   @version    1.0
*/
int8_t nm_bus_deinit(void) {
    return 0;
}

/*
*   @fn         nm_bus_reinit
*   @brief      re-initialize the bus wrapper
*   @param [in] void *config
*                   re-init configuration data
*   @return     M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*   @author     Dina El Sissy
*   @date       19 Sept 2012
*   @version    1.0
*/
int8_t nm_bus_reinit(void *config) {
    return M2M_SUCCESS;
}
