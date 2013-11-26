# Chopstx make rules.

CSRC += $(CHOPSTX)/entry.c $(CHOPSTX)/chopstx.c

ifneq ($(USE_EVENTFLAG),)
CSRC += $(CHOPSTX)/eventflag.c
endif

INCDIR += $(CHOPSTX)

BUILDDIR = build
OUTFILES = $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).bin
ifneq ($(ENABLE_OUTPUT_HEX),)
OUTFILES += $(BUILDDIR)/$(PROJECT).hex
endif


OPT += -ffunction-sections -fdata-sections -fno-common

OBJS    = $(addprefix $(BUILDDIR)/, $(notdir $(CSRC:.c=.o)))

IINCDIR   = $(patsubst %,-I%,$(INCDIR))
LLIBDIR   = $(patsubst %,-L%,$(LIBDIR))

VPATH     = $(sort $(dir $(CSRC)))
###
MCFLAGS   = -mcpu=$(MCU)

CFLAGS    = $(MCFLAGS) $(OPT) $(CWARN) -Wa,-alms=$(BUILDDIR)/$(notdir $(<:.c=.lst)) $(DEFS)

LDFLAGS = $(MCFLAGS) -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch,--gc-sections $(LLIBDIR)

CFLAGS   += -mthumb -mno-thumb-interwork -DTHUMB
LDFLAGS  += -mthumb -mno-thumb-interwork

CFLAGS   += -MD -MP -MF .dep/$(@F).d

all: $(OUTFILES)

$(OBJS): | $(BUILDDIR)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(OBJS) : $(BUILDDIR)/%.o : %.c Makefile
	@echo
	$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@

%.elf: $(OBJS) $(OBJS_ADD) $(LDSCRIPT)
	@echo
	$(LD) $(OBJS) $(OBJS_ADD) $(LDFLAGS) $(LIBS) -o $@

%.bin: %.elf $(LDSCRIPT)
	$(OBJCOPY) -O binary $< $@

%.hex: %.elf $(LDSCRIPT)
	$(OBJCOPY) -O ihex $< $@

clean:
	-rm -f -r .dep $(BUILDDIR)

# Include dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
