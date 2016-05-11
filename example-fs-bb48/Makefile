# Makefile for example application of Chopstx

PROJECT = sample

### Currently, it's for FS-BB48.

CHOPSTX = ..
LDSCRIPT= sample.ld
CSRC = sample.c first-pages.c usb_kl27z.c usb-cdc.c adc_kl27z.c

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m0plus
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DFREE_STANDING -DMHZ=48
OPT   = -O3 -Os -g
LIBS  =

####################
include ../rules.mk

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

distclean: clean
	rm -f board.h
