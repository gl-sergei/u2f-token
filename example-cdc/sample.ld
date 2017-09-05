/*
 * ST32F103 memory setup.
 */

MEMORY
{
    flash0 : org = 0x08000000, len = 4k
    flash  : org = 0x08000000+0x1000, len = 60k
    ram : org = 0x20000000, len = 20k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = 20k;
__ram_end__             = __ram_start__ + __ram_size__;

SECTIONS
{
    . = 0;

    .sys : ALIGN(4) SUBALIGN(4)
    {
	_sys = .;
	KEEP(*(.vectors))
	. = ALIGN(16);
	KEEP(*(.sys.version))
	KEEP(*(.sys.board_id))
	KEEP(*(.sys.board_name))
	build/sys-*.o(.text)
	build/sys-*.o(.text.*)
	build/sys-*.o(.rodata)
	build/sys-*.o(.rodata.*)
	. = ALIGN(1024);
	*(.sys.0)
	*(.sys.1)
	*(.sys.2)
    } > flash0

    _text = .;

    .startup : ALIGN(128) SUBALIGN(128)
    {
        KEEP(*(.startup.vectors))
        . = ALIGN (16);
    } > flash =0xffffffff

    .text : ALIGN(16) SUBALIGN(16)
    {
        *(.text.startup.*)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
	. = ALIGN(8);
    } > flash

    .ARM.extab : {*(.ARM.extab* .gnu.linkonce.armextab.*)} > flash

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash

    .eh_frame_hdr : {*(.eh_frame_hdr)} > flash

    .eh_frame : ONLY_IF_RO {*(.eh_frame)} > flash

    .textalign : ONLY_IF_RO { . = ALIGN(8); } > flash

    _etext = .;
    _textdata = _etext;

    .process_stack :
    {
        . = ALIGN(8);
        *(.process_stack.3)
        *(.process_stack.2)
        *(.process_stack.1)
        *(.process_stack.0)
        . = ALIGN(8);
    } > ram

    .main_stack :
    {
        . = ALIGN(8);
        *(.main_stack)
        . = ALIGN(8);
    } > ram

    .data :
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        . = ALIGN(4);
        *(.data.*)
        . = ALIGN(4);
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > ram AT > flash

    .bss :
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        . = ALIGN(4);
        *(.bss.*)
        . = ALIGN(4);
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
    } > ram

    PROVIDE(end = .);
    _end            = .;
}

__heap_base__   = _end;
__heap_end__    = __ram_end__;
