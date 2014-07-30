# Makefile for example application of Chopstx

PROJECT = sample

CHOPSTX = ..
LDSCRIPT= sample.ld
CSRC = sys.c aes-constant-ft.c sample.c

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

# MCU   = cortex-m3
MCU   = cortex-m0 # -save-temps
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DHAVE_SYS_H -DFREE_STANDING -DMHZ=48
# DEFS  = -DFREE_STANDING -DHAVE_SYS_H -DBUSY_LOOP -DCHX_FLAGS_MAIN=CHOPSTX_SCHED_RR
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
