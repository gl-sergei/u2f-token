# Makefile for example application of Chopstx

PROJECT = sample

### Currently, it's for STM32F0 Discovery.
### Please change lines started with '###' for Cortex-M3 board.

CHOPSTX = ..
LDSCRIPT= sample.ld
### LDSCRIPT= sample.ld.m3
CSRC = sample.c

CHIP=stm32f0
USE_SYS = yes

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

### MCU   = cortex-m3
MCU   = cortex-m0
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DUSE_SYS3 -DFREE_STANDING -DMHZ=48
### DEFS  = -DFREE_STANDING -DUSE_SYS3 -DBUSY_LOOP -DCHX_FLAGS_MAIN=CHOPSTX_SCHED_RR
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
