# Makefile for NeuG

# Define project name here
PROJECT = sample-msc

CHOPSTX = ..
FRAUCHEKY = ../../fraucheky
LDSCRIPT=

CSRC = main.c
CHIP=gnu-linux
USE_SYS =
USE_USB = yes
EMULATION=yes
# USE_ADC = yes

include $(FRAUCHEKY)/src.mk

###################################
CROSS =
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = none
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DGNU_LINUX_EMULATION
OPT   = -O3 -g
LIBS  = -lpthread

BFDNAME_OBJ=elf64-x86-64
BFDARCH=i386:x86-64

#######################
include $(CHOPSTX)/rules.mk

include $(FRAUCHEKY)/build.mk
###################################

board.h:
	@echo Please run configure to have a symbolic link \'board.h\'
	@exit 1

distclean:: clean
	-rm -f config.h
