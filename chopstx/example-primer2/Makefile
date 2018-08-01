# Makefile for example application of Chopstx

PROJECT = lcd

CHOPSTX = ..
NEUGSRC = ./neug/src
LDSCRIPT= lcd.ld
CSRC = primer2-switches.c primer2-ts.c lcd.c main.c \
       neug.c sha256.c

CHIP=stm32f103
USE_SYS = yes
USE_USB = yes
USE_ADC = yes

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy
MCU   = cortex-m3
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DUSE_SYS3 -DFREE_STANDING -DMHZ=48
# DEFS  = -DFREE_STANDING -DUSE_SYS3 -DBUSY_LOOP -DCHX_FLAGS_MAIN=CHOPSTX_SCHED_RR
OPT   = -O3 -Os -g
INCDIR = $(NEUGSRC)
LIBS  =

####################
include ../rules.mk

board.h: ../board/board-stm32-primer2.h neug
	ln -s ../board/board-stm32-primer2.h ./board.h

sys.c: board.h

neug:
	@echo Please make a symbolic link \'neug\' to the neug directory;
	@exit 1

adc_stm32f103.c neug.c sha256.c:
	ln -s $(NEUGSRC)/$@ $@

distclean: clean
	rm -f board.h neug adc_stm32f103.c neug.c sha256.c
