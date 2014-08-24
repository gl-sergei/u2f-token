/*
 * ST32F103 memory setup.
 */
__main_stack_size__     = 0x0100;  /* Exception handlers     */
__process0_stack_size__  = 0x0100;  /* Main program           */
__process1_stack_size__  = 0x0100; /* first thread program */
__process2_stack_size__  = 0x0100; /* second thread program */
__process3_stack_size__  = 0x0100; /* third thread program */

MEMORY
{
/*
    flash0 : org = 0x08000000, len = 4k
    flash  : org = 0x08000000+0x1000, len = 60k
*/
    flash0 : org = 0x08000000, len = 1k
    flash  : org = 0x08000000+0x0400, len = 60k
    ram : org = 0x20000000, len = 20k
}

__flash_start__ = 0x08001000;
__flash_end__   = 0x08020000;

__ram_start__           = ORIGIN(ram);
__ram_size__            = 20k;
__ram_end__             = __ram_start__ + __ram_size__;

SECTIONS
{
    . = 0;

    .sys : ALIGN(16) SUBALIGN(16)
    {
        _sys = .;
        KEEP(*(.vectors))
	. = ALIGN(16);
	*(.sys.version)
	build/sys.o(.text)
	build/sys.o(.text.*)
        build/sys.o(.rodata)
	build/sys.o(.rodata.*)
	. = ALIGN(1024);
/*
	*(.sys.0)
	*(.sys.1)
	*(.sys.2)
*/
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

    .vectors_in_ram :
    {
        . = ALIGN(8);
        __vector_ram_addr__ = .;
	KEEP(*(.bss.startup.*))
    } > ram

    .process_stack :
    {
        . = ALIGN(8);
        __process3_stack_base__ = .;
        . += __process3_stack_size__;
        . = ALIGN(8);
        __process_stack3_end__ = .;
        __process2_stack_base__ = .;
        . += __process2_stack_size__;
        . = ALIGN(8);
        __process_stack2_end__ = .;
        __process1_stack_base__ = .;
        . += __process1_stack_size__;
        . = ALIGN(8);
        __process_stack1_end__ = .;
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
