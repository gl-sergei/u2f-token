# U2F firmware

PROJECT = u2f

CHOPSTX = ../chopstx

ifeq ($(TARGET),)
	TARGET=MAPLE_MINI
endif

ifeq ($(TARGET),MAPLE_MINI)
	MCU      = cortex-m3
	CHIP     = stm32f103
	DRIVERS  = pbt.c
	DEFS     = -DUSE_SYS3 -DFREE_STANDING -DHAVE_PUSH_BUTTON -DTARGET_MAPLE_MINI
	LDSCRIPT = stm32f103.ld
	BOARD    = $(CHOPSTX)/board/board-maple-mini.h
else ifeq ($(TARGET),ST_DONGLE)
	MCU      = cortex-m3
	CHIP     = stm32f103
	DRIVERS  = pbt.c
	DEFS     = -DUSE_SYS3 -DFREE_STANDING -DTARGET_ST_DONGLE -DHAVE_PUSH_BUTTON
	LDSCRIPT = stm32f103.ld
	BOARD    = $(CHOPSTX)/board/board-st-dongle.h
else ifeq ($(TARGET),BLUE_PILL)
	MCU      = cortex-m3
	CHIP     = stm32f103
	DRIVERS  = uvoid.c
	DEFS     = -DUSE_SYS3 -DFREE_STANDING -DBLUE_PILL
	LDSCRIPT = stm32f103.ld
	BOARD    = $(CHOPSTX)/board/board-blue-pill.h
else ifeq ($(TARGET),BLACK_PILL)
	MCU      = cortex-m3
	CHIP     = stm32f103
	DRIVERS  = uvoid.c
	DEFS     = -DUSE_SYS3 -DFREE_STANDING -DBLACK_PILL
	LDSCRIPT = stm32f103.ld
	BOARD    = $(CHOPSTX)/board/board-black-pill.h
else ifeq ($(TARGET),TOMU)
	MCU      = cortex-m0plus
	CHIP     = efm32hg
	DRIVERS  = csn.c
	DEFS     = -DMAKE_ENTRY_PUBLIC -DUSE_SYS3 -DFREE_STANDING -DMHZ=21 \
	           -DHAVE_CAPSENSE -DTARGET_TOMU
	LDSCRIPT = efm32hg.ld
	BOARD    = $(CHOPSTX)/board/board-tomu.h
endif

ifeq ($(ENFORCE_DEBUG_LOCK),1)
DEFS += -DENFORCE_DEBUG_LOCK
endif

CSRC = u2f.c usb-hid.c dbug.c u2f-hid.c u2f-apdu.c \
	sha256.c neug.c random.c ec_p256r1.c bn.c jpc_p256r1.c \
	call-ec_p256r1.c modp256r1.c mod.c hmac.c platform.c $(DRIVERS)

USE_SYS = yes
USE_USB = yes
USE_ADC = yes
USE_EVENTFLAG = yes

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

CWARN = -Wall -Wextra -Wstrict-prototypes
OPT   = -O3 -Os -g
LIBS  =

####################
include $(CHOPSTX)/rules.mk

board.h:
	ln -s $(BOARD) board.h

cert/certificates.c:
	cd cert && ./gen.sh && cd ..

sys.c: board.h
u2f.c: board.h

u2f-apdu.c: cert/certificates.c

distclean: clean
	rm -f board.h

certclean:
	rm -f cert/certificates.c
	rm -f cert/opnssl.cnf
	rm -f cert/*.der
	rm -f cert/*.pem
	rm -f cert/*.csr
