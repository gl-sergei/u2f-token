# Makefile for example application of Chopstx

PROJECT = sample

CHOPSTX = ..
LDSCRIPT= sample.ld
CSRC = sys.c aes-constant-ft.c sample.c usb-cdc.c usb_stm32f103.c

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m3
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DFREE_STANDING
OPT   = -O3 -Os -g
LIBS  =

####################
include ../rules.mk

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

sys.c: board.h

distclean: clean
	rm -f board.h
