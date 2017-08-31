OUTPUT_ARCH(arm)
 
MEMORY
{
    flash (rx)  : ORIGIN = 0x00400000, LENGTH = 1M,   /* Restrict image to single bank */
    sram  (rwx) : ORIGIN = 0x20000000, LENGTH = 160K
}

SECTIONS
{
    .text :
    {
	. = ALIGN(4);
        KEEP(*(.flash_isr_vector))
        *(.text*)
        *(.rodata*)
	. = ALIGN(4);
    } > flash

    . = ALIGN(4);
    _etext = .;
    PROVIDE (__etext = .);

    .data : AT (__etext)
    {
	. = ALIGN(4);
	PROVIDE (__data_start = .);
	KEEP(*(.sram_isr_vector))
	*(.data*)
	KEEP(*(.sram_function))
        _edata = .;
	PROVIDE(__data_end = .);
    } > sram

    .bss :
    {
	 __bss_start = .;
	 *(.bss*)
	 *(COMMON)
	 . = ALIGN(8);
	 __bss_end = .;
	 . += 4K;
	 __exception_stack = .;
	 . += 4K;
	 __main_stack = .;
	 __heap_start = .;
    } > sram
}
end = .;
