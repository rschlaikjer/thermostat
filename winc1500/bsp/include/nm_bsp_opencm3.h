#ifndef _NM_BSP_OPENCM3_H_
#define _NM_BSP_OPENCM3_H_

#include <stdint.h>
#include <unistd.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include <nest/realtime.hpp>
#include <nest/interrupts.hpp>

extern uint32_t gWincCSPort;
extern uint16_t gWincCSPin;

extern uint32_t gWincResetPort;
extern uint16_t gWincResetPin;

extern uint32_t gWincIntPort;
extern uint16_t gWincIntPin;

extern uint32_t gWincENPort;
extern uint16_t gWincENPin;

#ifdef __cplusplus
extern "C" {
#endif
void winc_interrupt_bridge(void);
#ifdef __cplusplus
}
#endif

#endif /* _NM_BSP_OPENCM3_H_ */
