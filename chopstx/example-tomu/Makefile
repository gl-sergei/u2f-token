# Example application for Tomu board

PROJECT = sample

CHOPSTX = ..
LDSCRIPT= sample.ld
CSRC = sample.c usb-cdc.c

CHIP=efm32hg
USE_SYS = yes
USE_ADC = yes
USE_USB = yes

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

### MCU   = cortex-m3
MCU   = cortex-m0plus
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DMAKE_ENTRY_PUBLIC -DUSE_SYS3 -DFREE_STANDING -DMHZ=14
OPT   = -O3 -Os -g
# OPT   = -O1 -g
LIBS  =

####################
include ../rules.mk

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

sys.c: board.h

distclean: clean
	rm -f board.h
