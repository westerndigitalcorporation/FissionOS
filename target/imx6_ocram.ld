OUTPUT_ARCH(arm)

MEMORY
{
    sram  (rwx) : ORIGIN = 0x00900000, LENGTH = 128K
}

SECTIONS
{
    .text :
    {
	. = ALIGN(4);
        KEEP(*(.startup))
        *(.text*)
    } > sram

    . = ALIGN(4);
    _text_end = .;
    PROVIDE (__text_end = .);

    .data : AT (__text_end)
    {
	. = ALIGN(4);
        *(.rodata*)
	*(.data*)
    }

    . = ALIGN(4);
    __data_end = .;
    PROVIDE(__data_end = .);

    .bss : AT (__data_end)
    {
	. = ALIGN(4);
	__bss_start = .;
	*(.bss*)
	*(COMMON)
	. = ALIGN(4);
	__bss_end = .;
	__heap_start = .;
    } > sram

    . = 0x0091fff0;
    __stack = .;

    __memtest_state = .;

    . += 4;
    __memtest_addr = .;

    . += 4;
    __memtest_passes = .;
}
end = .;
