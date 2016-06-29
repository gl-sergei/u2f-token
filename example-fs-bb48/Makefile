# Makefile for example application of Chopstx

PROJECT = sample

### Currently, it's for FS-BB48.

CHOPSTX = ..
LDSCRIPT= sample.ld
CSRC = sample.c usb-cdc.c command.c touch.c
CHIP=mkl27z

USE_SYS = yes
USE_USB = yes
USE_ADC = yes

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m0plus
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DFREE_STANDING -DMHZ=48 -DUSE_SYS3
OPT   = -O3 -Os -g
LIBS  =

####################
include ../rules.mk

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

distclean: clean
	rm -f board.h
