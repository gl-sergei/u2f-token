/*
 * ST32F103 memory setup.
 */
__main_stack_size__      = 0x0100; /* Idle+Exception handlers */
__process0_stack_size__  = 0x0400; /* Main program            */
__process1_stack_size__  = 0x0200; /* pwd                     */
__process2_stack_size__  = 0x0200; /* blk                     */
__process3_stack_size__  = 0x0200; /* usb-hid                 */
__process4_stack_size__  = 0x1000; /* u2f-hid                 */
__process5_stack_size__  = 0x0500; /* rng                     */
__process6_stack_size__  = 0x0200; /* pbt                     */

MEMORY
{
    flash0 : org = 0x08000000, len = 4k
    flash  : org = 0x08000000+0x1000, len = 58k
    flash1 : org = 0x08000000+0xf800, len = 2k
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

        __process6_stack_base__ = .;
        . += __process6_stack_size__;
        . = ALIGN(8);
        __process6_stack_end__ = .;

        __process5_stack_base__ = .;
        . += __process5_stack_size__;
        . = ALIGN(8);
        __process5_stack_end__ = .;

        __process4_stack_base__ = .;
        . += __process4_stack_size__;
        . = ALIGN(8);
        __process4_stack_end__ = .;

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

    .flash_storage :
    {
        . = ALIGN (1024);
        _device_key_base = .;
        *(.device.key);
        . = ALIGN (1024);
        _auth_ctr_base = .;
        *(.auth.ctr);
        . = ALIGN (1024);
    } > flash1

    PROVIDE(end = .);
    _end            = .;
}

__heap_base__   = _end;
__heap_end__    = __ram_end__;
