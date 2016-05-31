/*
 * MK27Z memory setup.
 */
__main_stack_size__      = 0x0100; /* Idle+Exception handlers */
__process0_stack_size__  = 0x0300; /* Main program            */
__process1_stack_size__  = 0x0200; /* first thread program    */
__process2_stack_size__  = 0x0100; /* second thread program   */
__process3_stack_size__  = 0x0200; /* third thread program    */

MEMORY
{
    flash  : org = 0x00000000, len = 256k
    ram : org = 0x1fffe000, len = 32k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = 32k;
__ram_end__             = __ram_start__ + __ram_size__;

SECTIONS
{
    . = 0;

    _text = .;

    .text : ALIGN(16) SUBALIGN(8)
    {
        KEEP(*(.first_page.first_words))
        KEEP(*(.sys.version))
        KEEP(*(.sys.board_info))
        KEEP(*(.sys.vectors))
	build/sys-*.o(.text)
	build/sys-*.o(.text.*)
	build/sys-*.o(.rodata)
	build/sys-*.o(.rodata.*)
	. = ALIGN(1024);
        KEEP(*(.flash_config))
	KEEP(*(.fixed_function.reset))
	KEEP(*(.fixed_function.flash_do_internal))
	KEEP(*(.fixed_function.flash_do))
	KEEP(*(.fixed_function.flash_erase_page))
	KEEP(*(.fixed_function.flash_program_word))
	KEEP(*(.fixed_function.crc32_init))
	KEEP(*(.fixed_function.crc32_u8))
	KEEP(*(.fixed_function.crc32_u32))
	/*
	 * Because of alignment requirement
	 * of startup.vectors, align to 256.
	 */
	. = ALIGN(256);
        KEEP(*(.crc32_table))
        KEEP(*(.startup.vectors))
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
    } > flash =0xffffffff

    /DISCARD/ :
    {
        *(.bss.startup.0)
    }

    .ARM.extab : {*(.ARM.extab* .gnu.linkonce.armextab.*)} > flash

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash =0xffffffff

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
    } > ram AT > flash =0xffffffff

    .bss :
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        . = ALIGN(4);
        *(.bss.*)
        . = ALIGN(4);
        *(COMMON)
        . = ALIGN(512);
	__usb_bdt__ = .;
        . += 512;
        PROVIDE(_bss_end = .);
    } > ram

    PROVIDE(end = .);
    _end            = .;
}

__heap_base__   = _end;
__heap_end__    = __ram_end__;
