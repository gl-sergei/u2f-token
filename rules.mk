# Chopstx make rules.

CSRC += $(CHOPSTX)/entry.c $(CHOPSTX)/chopstx.c

ifneq ($(USE_EVENTFLAG),)
CSRC += $(CHOPSTX)/eventflag.c
endif

ifneq ($(USE_SYS),)
CSRC += $(CHOPSTX)/mcu/sys-$(CHIP).c
endif
ifneq ($(USE_USB),)
ifeq ($(EMULATION),)
CSRC += $(CHOPSTX)/mcu/usb-$(CHIP).c
else
CSRC += $(CHOPSTX)/mcu/usb-usbip.c
endif
endif
ifneq ($(USE_ADC),)
CSRC += $(CHOPSTX)/contrib/adc-$(CHIP).c
endif

INCDIR += $(CHOPSTX)

BUILDDIR = build
ifeq ($(EMULATION),)
OUTFILES = $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).bin
ifneq ($(ENABLE_OUTPUT_HEX),)
OUTFILES += $(BUILDDIR)/$(PROJECT).hex
endif
else
OUTFILES  = $(BUILDDIR)/$(PROJECT)
endif


OPT += -ffunction-sections -fdata-sections -fno-common

OBJS    = $(addprefix $(BUILDDIR)/, $(notdir $(CSRC:.c=.o)))

IINCDIR   = $(patsubst %,-I%,$(INCDIR))
LLIBDIR   = $(patsubst %,-L%,$(LIBDIR))

VPATH     = $(sort $(dir $(CSRC)))
###
ifeq ($(EMULATION),)
MCFLAGS   = -mcpu=$(MCU)
LDFLAGS   = $(MCFLAGS) -nostartfiles -T$(LDSCRIPT) \
            -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch,--gc-sections
else
MCFLAGS   =
LDFLAGS   =
DEFS      += -D_GNU_SOURCE
endif

CFLAGS    = $(MCFLAGS) $(OPT) $(CWARN) -Wa,-alms=$(BUILDDIR)/$(notdir $(<:.c=.lst)) $(DEFS)
LDFLAGS  += $(LLIBDIR)

ifeq ($(EMULATION),)
CFLAGS   += -mthumb -mno-thumb-interwork -DTHUMB
LDFLAGS  += -mthumb -mno-thumb-interwork
endif

CFLAGS   += -MD -MP -MF .dep/$(@F).d

all: $(OUTFILES)

$(OBJS): | $(BUILDDIR)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(OBJS) : $(BUILDDIR)/%.o : %.c Makefile
	@echo
	$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@

ifeq ($(EMULATION),)
%.elf: $(OBJS) $(OBJS_ADD) $(LDSCRIPT)
	@echo
	$(LD) $(OBJS) $(OBJS_ADD) $(LDFLAGS) $(LIBS) -o $@

%.bin: %.elf $(LDSCRIPT)
	$(OBJCOPY) -O binary $< $@

%.hex: %.elf $(LDSCRIPT)
	$(OBJCOPY) -O ihex $< $@
else
$(BUILDDIR)/$(PROJECT): $(OBJS) $(OBJS_ADD)
	@echo
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(OBJS_ADD) $(LIBS)
endif

clean:
	-rm -f -r .dep $(BUILDDIR)

# Include dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
