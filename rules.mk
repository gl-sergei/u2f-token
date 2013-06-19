# Chopstx make rules.

CSRC += $(CHOPSTX)/entry.c $(CHOPSTX)/chopstx.c

ifneq ($(USE_EVENTFLAG),)
CSRC += $(CHOPSTX)/eventflag.c
endif

INCDIR _= $(CHOPSTX)

BUILDDIR = build
OUTFILES = $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).bin

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

all: $(OBJS) $(OUTFILES)

$(OBJS): | $(BUILDDIR)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(OBJS) : $(BUILDDIR)/%.o : %.c Makefile
	@echo
	$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@

%.elf: $(OBJS) $(LDSCRIPT)
	@echo
	$(LD) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%.bin: %.elf $(LDSCRIPT)
	$(OBJCOPY) -O binary $< $@

clean:
	-rm -f -r .dep $(BUILDDIR)

# Include dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
