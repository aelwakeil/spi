# BINARY = paper paper_init spi_driver uart_driver
# 
 LDSCRIPT = ../stm32vl-discovery.ld
 #LDFLAGS += -Wl,-Ttext=0x8002000
 CFLAGS += -std=c99
# 
# 
# include ../Makefile.include

# Sources

SRCS = paper.c paper_init.c spi_driver.c uart_driver.c

# Project name

PROJ_NAME=paper
OUTPATH=build

###################################################

# Check for valid float argument
# NOTE that you have to run make clan after
# changing these as hardfloat and softfloat are not
# binary compatible
ifneq ($(FLOAT_TYPE), hard)
ifneq ($(FLOAT_TYPE), soft)
override FLOAT_TYPE = hard
#override FLOAT_TYPE = soft
endif
endif

###################################################

PREFIX	?= arm-none-eabi
#PREFIX		?= arm-elf
CC		= $(PREFIX)-gcc
LD		= $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
GDB		= $(PREFIX)-gdb
SIZE		= $(PREFIC)-size
STM32_DIR = ../stm32f10x
OPENCM3_DIR = ../libopencm3
TOOLCHAIN_DIR ?= ../libopencm3
ifeq ($(wildcard ../libopencm3/lib/libopencm3_stm32f1.a),)
ifneq ($(strip $(shell which $(CC))),)
TOOLCHAIN_DIR := $(shell dirname `which $(CC)`)/../$(PREFIX)
endif
else
ifeq ($(V),1)
$(info We seem to be building the example in the source directory. Using local library!)
endif
endif

ARCH_FLAGS      = -mthumb -mcpu=cortex-m3 -msoft-float
CFLAGS		+= -Os -g \
		   -Wall -Wextra -Wimplicit-function-declaration \
		   -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes \
		   -Wundef -Wshadow \
		   -I$(TOOLCHAIN_DIR)/include \
		   -fno-common $(ARCH_FLAGS) -MD -DSTM32F1
LDSCRIPT	?= $(BINARY).ld
LDFLAGS		+= --static -Wl,--start-group -lc -lgcc -Wl,--end-group \
		   -L$(TOOLCHAIN_DIR)/lib \
		   -T$(LDSCRIPT) -nostartfiles -Wl,--gc-sections \
		   $(ARCH_FLAGS) -mfix-cortex-m3-ldrd
#-lnosys
ifneq ($(OPENCM3_DIR),)
CFLAGS          += -I$(OPENCM3_DIR)/include
LDFLAGS         += -L$(OPENCM3_DIR)/lib -L$(OPENCM3_DIR)/lib/stm32/f1
SCRIPT_DIR      = $(OPENCM3_DIR)/share
else
SCRIPT_DIR      = $(shell dirname $(shell readlink -f $(shell which $(PREFIX)-gcc)))/../$(PREFIX)/share
endif


CFLAGS          += -I$(STM32_DIR)/src
LDFLAGS         += -L$(STM32_DIR)/inc
LDFLAGS         += -L$(STM32_DIR)
# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
else
LDFLAGS += -Wl,--print-gc-sections
endif
#####################################################


# 
# BINPATH=~/sat/bin
# CC=$(BINPATH)/arm-none-eabi-gcc
# OBJCOPY=$(BINPATH)/arm-none-eabi-objcopy
# SIZE=$(BINPATH)/arm-none-eabi-size
# 
# CFLAGS  = -std=gnu99 -g -O2 -Wall -Tstm32_flash.ld
# CFLAGS += -mlittle-endian -mthumb -mthumb-interwork -nostartfiles -mcpu=cortex-m4
# 
# ifeq ($(FLOAT_TYPE), hard)
# CFLAGS += -fsingle-precision-constant -Wdouble-promotion
# CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
# #CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
# else
# CFLAGS += -msoft-float
# endif

###################################################

vpath %.c src
#vpath %.a lib

ROOT=$(shell pwd)
# 
# CFLAGS += -Iinc -Ilib -Ilib/inc 
# CFLAGS += -Ilib/inc/core -Ilib/inc/peripherals 

## SRCS += lib/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: clean lib proj
	$(SIZE) $(OUTPATH)/$(PROJ_NAME).elf

lib:
	@#$(MAKE) -C lib FLOAT_TYPE=$(FLOAT_TYPE)

proj: 	$(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).bin

$(OUTPATH)/$(PROJ_NAME).bin: $(OUTPATH)/$(PROJ_NAME).elf
	@#printf "  OBJCOPY $(OUTPATH)/$(PROJ_NAME).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).bin

$(OUTPATH)/$(PROJ_NAME).hex: $(OUTPATH)/$(PROJ_NAME).elf
	@#printf "  OBJCOPY $(OUTPATH)/$(PROJ_NAME).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).hex

$(OUTPATH)/$(PROJ_NAME).srec: $(OUTPATH)/$(PROJ_NAME).elf
	@#printf "  OBJCOPY $(OUTPATH)/$(PROJ_NAME).srec\n"
	$(Q)$(OBJCOPY) -Osrec $(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).srec

$(OUTPATH)/$(PROJ_NAME).list: $(OUTPATH)/$(PROJ_NAME).elf
	@#printf "  OBJDUMP $(OUTPATH)/$(PROJ_NAME).list\n"
	$(Q)$(OBJDUMP) -S $(OUTPATH)/$(PROJ_NAME).elf > $(OUTPATH)/$(PROJ_NAME).list

$(OUTPATH)/$(PROJ_NAME).elf: $(OBJS) $(LDSCRIPT) $(TOOLCHAIN_DIR)/lib/libopencm3_stm32f1.a
	@#printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) -o $(OUTPATH)/$(PROJ_NAME).elf $(OBJS) -lopencm3_stm32f1 $(LDFLAGS)

$(OUTPATH)/$(PROJ_NAME).o: $(SRCS) Makefile
	@#printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<
clean:
	$(Q)rm -f *.o
	$(Q)rm -f *.d
	$(Q)rm -f $(OUTPATH)/$(PROJ_NAME).srec
	$(Q)rm -f $(OUTPATH)/$(PROJ_NAME).list
	$(Q)rm -f $(OUTPATH)/$(PROJ_NAME).elf
	$(Q)rm -f $(OUTPATH)/$(PROJ_NAME).hex
	$(Q)rm -f $(OUTPATH)/$(PROJ_NAME).bin
	
