OUTPUT_ARCH(arm)

ENTRY(reset_handler)

MEMORY
{
    sram   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K,
    lpsram (rwx) : ORIGIN = 0x30000000, LENGTH = 8K
}

SECTIONS
{
    .text :
    {
	KEEP(*(.sram_isr_vector))
        *(.text*)
        *(.rodata*)
    } > sram

    . = ALIGN(4);
    _etext = .;
    PROVIDE (__etext = .);

    .data : AT (__etext)
    {
	. = ALIGN(4);
	PROVIDE (__data_start = .);
	KEEP(*(.sram_function))
	*(.data*)
        _edata = .;
	PROVIDE(__data_end = .);
    } > sram

    .bss :
    {
	. = ALIGN(4);
	 __bss_start = .;
	 *(.bss*)
	 *(COMMON)
	 __bss_end = .;
    } > sram

    .stacks :
    {
	 . = ALIGN(8);
	 . += 2K;
	 __exception_stack = .;
	 . += 4K;
	 __main_stack = .;
	 . = ALIGN(8);
	 __heap_start = .;
    } > lpsram

}
end = .;
