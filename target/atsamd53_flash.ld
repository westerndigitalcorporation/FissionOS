OUTPUT_ARCH(arm)

ENTRY(reset_handler)

MEMORY
{
    flash  (rx)  : ORIGIN = 0x00000000, LENGTH = 1M,
    sram   (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

SECTIONS
{
    .text :
    {
	. = ALIGN(4);
        KEEP(*(.flash_isr_vector))
        *(.text*)
    } > flash

    . = ALIGN(4);
    _etext = .;
    PROVIDE (__etext = .);

    .data : AT(__etext)
    {
	. = ALIGN(4);
	PROVIDE (__data_start = .);
	KEEP(*(.sram_isr_vector))
        *(.rodata*)     /* Const data must be in SRAM for USB to see it */
	*(.data*)
	KEEP(*(.sram_function))
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
	 . += 4K;
	 __exception_stack = .;
	 . += 4K;
	 __main_stack = .;
	 . = ALIGN(8);
	 __heap_start = .;
    } > sram

}
end = .;
