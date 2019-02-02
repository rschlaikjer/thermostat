#ifndef _NM_BUS_WRAPPER_OPENCM3_H_
#define _NM_BUS_WRAPPER_OPENCM3_H_

#include "bsp/include/nm_bsp.h"
#include "bsp/include/nm_bsp_opencm3.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include <nest/spi.hpp>

#define NM_BUS_MAX_TRX_SZ   256
#define SPI_TIMEOUT_MS   100

int spi_xfer8(uint8_t write, uint8_t *read);

#endif	/*_NM_BUS_WRAPPER_OPENCM3_H_*/
