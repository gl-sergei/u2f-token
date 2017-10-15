# Makefile for example application of Chopstx

PROJECT = u2f

CHOPSTX = ..
LDSCRIPT= u2f.ld
CSRC = u2f.c usb-hid.c dbug.c u2f-hid.c u2f-apdu.c \
	sha256.c neug.c random.c ec_p256r1.c bn.c jpc_p256r1.c \
	call-ec_p256r1.c modp256r1.c mod.c hmac.c csn.c

CHIP=efm32

USE_SYS = yes
USE_USB = yes
USE_ADC = yes
USE_EVENTFLAG = yes

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m0plus
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DMAKE_ENTRY_PUBLIC -DUSE_SYS3 -DFREE_STANDING -DMHZ=21
OPT   = -O3 -Os -fstack-usage
LIBS  =

####################
include ../rules.mk

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

cert/certificates.c:
	cd cert && ./gen.sh && cd ..

sys.c: board.h

u2f-apdu.c: cert/certificates.c

distclean: clean
	rm -f board.h

certclean:
	rm -f cert/certificates.c
	rm -f cert/*.der
	rm -f cert/*.pem
	rm -f cert/*.csr
