# Makefile for example application of Chopstx

PROJECT = sample

### This is for GNU/Linux

CHOPSTX = ..
LDSCRIPT=
CSRC = sample.c usb-cdc.c command.c

CHIP=gnu-linux
USE_SYS = yes
USE_USB = yes
USE_ADC = yes
EMULATION=yes

###################################
CROSS =
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = none
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DGNU_LINUX_EMULATION -DUSE_SYS_BOARD_ID
OPT   = -g # -O3 -Os
LIBS  = -lpthread

####################
include ../rules.mk

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

distclean: clean

build/flash.data: Makefile
	@echo 'Generating 8192-byte flash.data'
	@/bin/echo -n -e '\xff\xff\xff\xff\xff\xff\xff\xff' >$@
	@for i in $(shell seq 1023); do /bin/echo -n -e '\xff\xff\xff\xff\xff\xff\xff\xff' >>$@; done


