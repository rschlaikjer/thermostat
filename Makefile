BINARY = nest

LDSCRIPT = stm32f0.ld

LIBNAME		= opencm3_stm32f0

DEFS		+= -DSTM32F0 -DCONF_WINC_DEBUG=0 -DCONF_WINC_USE_SPI -DCONF_PERIPH # -DWIFI_FIRMWARE_UPDATE_MODE
FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m0 $(FP_FLAGS) -Wl,--wrap=malloc --specs=nano.specs

BMP_PORT = /dev/ttyACM0

include rules.mk
