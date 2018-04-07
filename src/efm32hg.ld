/*
 * EFM32HG memory setup.
 */
__main_stack_size__      = 0x0200; /* Idle+Exception handlers */
__process0_stack_size__  = 0x0200; /* Main program            */
__process1_stack_size__  = 0x0100; /* pwd                     */
__process2_stack_size__  = 0x0100; /* blk                     */
__process3_stack_size__  = 0x0200; /* usb-hid                 */
__process4_stack_size__  = 0x0800; /* u2f-hid                 */
__process5_stack_size__  = 0x0200; /* rng                     */
__process6_stack_size__  = 0x0200; /* pbt / csn               */

MEMORY
{
    flash  : org = 0x00004000, len = 46k
    ram : org = 0x20000000, len = 8k
    flash1 : org = 0x00004000+0xb800, len = 2k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = 8k;
__ram_end__             = __ram_start__ + __ram_size__;

SECTIONS
{
    . = 0;

    _text = .;

    .text : ALIGN(16) SUBALIGN(8)
    {
    . = ALIGN(16);
        KEEP(*(.first_page.first_words))
    . = ORIGIN(flash) + 0x90;
        KEEP(*(.toboot.config))
        KEEP(*(.sys.vectors))
        *(.text.startup.*)
    build/sys-*.o(.text)
    build/sys-*.o(.text.*)
    build/sys-*.o(.rodata)
    build/sys-*.o(.rodata.*)
    /*
     * Because of alignment requirement
     * of startup.vectors, align to 256.
     */
    . = ALIGN(256);
        KEEP(*(.startup.vectors))
    . = ALIGN(16);
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
        . = ALIGN(512);
    __usb_bdt__ = .;
        . += 512;
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
