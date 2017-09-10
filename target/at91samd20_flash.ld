OUTPUT_ARCH(arm)

ENTRY(reset_handler)

MEMORY
{
    flash  (rx)  : ORIGIN = 0x00000000, LENGTH = 256K,
    sram   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.flash_isr_vector))
        *(.text*)
        *(.rodata*)
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
        . = ALIGN(4);
         __bss_start = .;
         *(.bss*)
         *(COMMON)
         __bss_end = .;
    } > sram

    .stacks :
    {
         . = ALIGN(8);
         . += 1K;
         __exception_stack = .;
         . += 1K;
         __main_stack = .;
         . = ALIGN(8);
         __heap_start = .;
    } > sram

}
end = .;
            
