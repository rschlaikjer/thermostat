#include "bsp/include/nm_bsp.h"
#include "bsp/include/nm_bsp_opencm3.h"
#include "common/include/nm_common.h"

uint32_t gWincCSPort = GPIOB;
uint16_t gWincCSPin = GPIO12;

uint32_t gWincResetPort = GPIOB;
uint16_t gWincResetPin = GPIO9;

uint32_t gWincIntPort = GPIOB;
uint16_t gWincIntPin = GPIO10;

uint32_t gWincENPort = GPIOB;
uint16_t gWincENPin = GPIO11;

static tpfNmBspIsr gpfIsr;

void exti4_15_isr(void) {
    M2M_PRINT("ISR!\r\n");
    if (exti_get_flag_status(gWincIntPin)) {
        exti_reset_request(gWincIntPin);
        if (gpfIsr) {
            gpfIsr();
        }
    }
}

/*
 *	@fn		init_chip_pins
 *	@brief	Initialize reset, chip enable and wake pin
 *	@author	M.S.M
 *	@date	11 July 2012
 *	@version	1.0
 */
static void init_chip_pins(void) {
    // Configure RESETN pin as output
    gpio_mode_setup(gWincResetPort, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, gWincResetPin);
    gpio_set(gWincResetPort, gWincResetPin);

	/* Configure INTN pins as input. */
    gpio_mode_setup(gWincIntPort, GPIO_MODE_INPUT, GPIO_PUPD_NONE, gWincIntPin);

	/* Configure CHIP_EN as pull-up */
    gpio_mode_setup(gWincENPort, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, gWincENPin);

    // Chip select
    gpio_mode_setup(gWincCSPort, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, gWincCSPin);
    gpio_set(gWincCSPort, gWincCSPin);
}

static void deinit_chip_pins(void) {
    // Deinit reset pin
    gpio_clear(gWincResetPort, gWincResetPin);
    gpio_mode_setup(gWincResetPort, GPIO_MODE_INPUT, GPIO_PUPD_NONE, gWincResetPin);

    // Deinit EN pin (remove pullup)
    gpio_mode_setup(gWincENPort, GPIO_MODE_INPUT, GPIO_PUPD_NONE, gWincENPin);
}

/*
 *	@fn		nm_bsp_init
 *	@brief	Initialize BSP
 *	@return	0 in case of success and -1 in case of failure
 *	@author	M.S.M
 *	@date	11 July 2012
 *	@version	1.0
 */
sint8 nm_bsp_init(void) {

	gpfIsr = NULL;

	init_chip_pins();

	nm_bsp_reset();

	return M2M_SUCCESS;
}

/**
 *	@fn		nm_bsp_deinit
 *	@brief	De-iInitialize BSP
 *	@return	0 in case of success and -1 in case of failure
 *	@author	M. Abdelmawla
 *	@date	11 July 2012
 *	@version	1.0
 */
sint8 nm_bsp_deinit(void) {
	deinit_chip_pins();
	return M2M_SUCCESS;
}

/**
 *	@fn		nm_bsp_reset
 *	@brief	Reset NMC1500 SoC by setting CHIP_EN and RESET_N signals low,
 *           CHIP_EN high then RESET_N high
 *	@author	M. Abdelmawla
 *	@date	11 July 2012
 *	@version	1.0
 */
void nm_bsp_reset(void) {
    gpio_clear(gWincResetPort, gWincResetPin);
    nm_bsp_sleep(100);
    gpio_set(gWincResetPort, gWincResetPin);
    nm_bsp_sleep(100);
}

/*
 *	@fn		nm_bsp_sleep
 *	@brief	Sleep in units of mSec
 *	@param[IN]	u32TimeMsec
 *				Time in milliseconds
 *	@author	M.S.M
 *	@date	28 OCT 2013
 *	@version	1.0
 */
void nm_bsp_sleep(uint32 u32TimeMsec) {
	n_sleep((uint64_t) u32TimeMsec);
}

/*
 *	@fn		nm_bsp_register_isr
 *	@brief	Register interrupt service routine
 *	@param[IN]	pfIsr
 *				Pointer to ISR handler
 *	@author	M.S.M
 *	@date	28 OCT 2013
 *	@sa		tpfNmBspIsr
 *	@version	1.0
 */
void nm_bsp_register_isr(tpfNmBspIsr pfIsr) {
	gpfIsr = pfIsr;

    // Trigger on rising edge of EXTI 10
    exti_set_trigger(EXTI10, EXTI_TRIGGER_RISING);

    // Configure EXTI10 to use GPIOB
    exti_select_source(EXTI10, GPIOB);

    // Enable exti interrupts & events
    exti_enable_request(EXTI10);

    // Enable the interrupt in the NVIC
    nvic_enable_irq(NVIC_EXTI4_15_IRQ);
}

/*
 *	@fn		nm_bsp_interrupt_ctrl
 *	@brief	Enable/Disable interrupts
 *	@param[IN]	u8Enable
 *				'0' disable interrupts. '1' enable interrupts
 *	@author	M.S.M
 *	@date	28 OCT 2013
 *	@version	1.0
 */
void nm_bsp_interrupt_ctrl(uint8 u8Enable) {
	if (u8Enable) {
        // Unmask the interrupt on those pins
        exti_enable_request(EXTI10);
        nvic_enable_irq(NVIC_EXTI4_15_IRQ);
	} else {
        // Unmask the interrupt on those pins
        exti_disable_request(EXTI10);
        nvic_disable_irq(NVIC_EXTI4_15_IRQ);
	}
}
