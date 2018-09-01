DEFS		+= -DSTM32F0
FP_FLAGS	?= -msoft-float

DEFS		+= -I$(OPENCM3_DIR)/include -I./
LDFLAGS		+= -L$(OPENCM3_DIR)/lib
LDLIBS		+= -l$(LIBNAME)
LDSCRIPT	?= $(BINARY).ld
ARCH_FLAGS	= -mthumb -mcpu=cortex-m0 $(FP_FLAGS)

###############################################################################
# C flags

TGT_CFLAGS	+= $(OPT) $(CSTD) $(DEBUG)
TGT_CFLAGS	+= $(ARCH_FLAGS)
TGT_CFLAGS	+= -Wextra -Wshadow -Wimplicit-function-declaration
TGT_CFLAGS	+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
TGT_CFLAGS	+= -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C++ flags

TGT_CXXFLAGS	+= $(OPT) $(CXXSTD) $(DEBUG)
TGT_CXXFLAGS	+= $(ARCH_FLAGS)
TGT_CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls  -Weffc++
TGT_CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C & C++ preprocessor common flags

TGT_CPPFLAGS	+= -MD
TGT_CPPFLAGS	+= -Wall -Wundef
TGT_CPPFLAGS	+= $(DEFS)

###############################################################################
# Linker flags

TGT_LDFLAGS		+= --static -nostartfiles
TGT_LDFLAGS		+= -T$(LDSCRIPT)
TGT_LDFLAGS		+= $(ARCH_FLAGS) $(DEBUG)
TGT_LDFLAGS		+= -Wl,-Map=$(*).map -Wl,--cref
TGT_LDFLAGS		+= -Wl,--gc-sections
ifeq ($(V),99)
TGT_LDFLAGS		+= -Wl,--print-gc-sections
endif

###############################################################################
# Used libraries

LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group -lm

OBJS	= ./socket/source/socket.o \
	./common/source/nm_common.o \
	./driver/source/m2m_periph.o \
	./driver/source/nmasic.o \
	./driver/source/m2m_ota.o \
	./driver/source/nmspi.o \
	./driver/source/nmbus.o \
	./driver/source/m2m_hif.o \
	./driver/source/m2m_ate_mode.o \
	./driver/source/nmdrv.o \
	./driver/source/m2m_wifi.o \
	./driver/source/nmuart.o \
	./driver/source/m2m_crypto.o \
	./driver/source/m2m_ssl.o \
	./driver/source/nmi2c.o \
	./spi_flash/source/spi_flash.o \
	./bsp/source/nm_bsp_opencm3.o \
	./bus_wrapper/source/nm_bus_wrapper_opencm3.o

# Compiler defines
PREFIX		?= arm-none-eabi

CC		:= $(PREFIX)-gcc
CXX		:= $(PREFIX)-g++
LD		:= $(PREFIX)-gcc
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
SIZE		:= $(PREFIX)-size
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
GDB		:= $(PREFIX)-gdb
STFLASH		= $(shell which st-flash)
STYLECHECK	:= /checkpatch.pl
STYLECHECKFLAGS	:= --no-tree -f --terse --mailback
STYLECHECKFILES	:= $(shell find . -name '*.[ch]')
OPT		:= -Os
DEBUG		:= -ggdb3
CSTD		?= -std=c99

# Libs
OPENCM3_DIR := ../libopencm3/

all: winc1500.a

winc1500.a: $(OBJS)
	ar cr winc1500.a $(OBJS)

%.o: %.c
	$(CC) $(TGT_CFLAGS) $(CFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $(*).o -c $(*).c
