/*
 * ST32F0 memory setup.
 */
__main_stack_size__     = 0x0100;  /* Idle+Exception handlers */
__process0_stack_size__  = 0x0100; /* Main program            */
__process1_stack_size__  = 0x0100; /* first thread program    */
__process2_stack_size__  = 0x0100; /* second thread program   */
__process3_stack_size__  = 0x0100; /* third thread program    */

MEMORY
{
    flash  : org = 0x08000000, len = 16k
    ram : org = 0x20000000, len = 4k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = 4k;
__ram_end__             = __ram_start__ + __ram_size__;

SECTIONS
{
    . = 0;

    _text = .;

    .text : ALIGN(16) SUBALIGN(16)
    {
	KEEP(*(.vectors))
	. = ALIGN(16);
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

    /DISCARD/ :
    {
        *(.startup.vectors)
        *(.bss.startup.0)
    }

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
        __process3_stack_base__ = .;
        . += __process3_stack_size__;
        . = ALIGN(8);
        __process3_stack_end__ = .;
        __process2_stack_base__ = .;
        . += __process2_stack_size__;
        . = ALIGN(8);
        __process2_stack_end__ = .;
        __process1_stack_base__ = .;
        . += __process1_stack_size__;
        . = ALIGN(8);
        __process1_stack_end__ = .;
        __process0_stack_base__ = .;
        . += __process0_stack_size__;
        . = ALIGN(8);
        __process0_stack_end__ = .;
    } > ram

    .main_stack :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
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
