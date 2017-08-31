/*
 * sam4_vectors.c
 *
 *
 * Copyright (c) 2013-2017 Western Digital Corporation or its affiliates.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the copyright holder nor the names of its contributors may not
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jeremy Garff <jeremy.garff@sandisk.com>
 *
 */


#include <stdint.h>

#include "vectors.h"


extern uint32_t __data_start, __data_end, __etext, __bss_start,
                __bss_end, __main_stack, __exception_stack;
extern struct vector_table sram_isr_vectors;

extern int main(int argc, char *argv[]);
void reset_handler(void)
{
    volatile uint32_t *src = &__etext, *dst = &__data_start;
    // Change the active stack to the main, but keep priviledged
    uint32_t control = (1 << 1);

    // Make sure the stackalign bit is set in the CCR.  This
    // ensures that interrupt/exceptions have properly aligned
    // stack references.  Note:  This only works on Cortex-M3
    // Rev1 and later silicon.
    SCB->ccr |= CCB_CCR_STKALIGN;

    // Set the process stack pointer register.  Note:  The ARM
    // documentation refers to the exception stack as the main
    // stack.  I found this confusing, so in the code the
    // interrupt/exception stack is called the exception stack
    // and the process stack is called the main stack.
    asm volatile ("msr psp, %0;"
                  :
                  : "r"(&__main_stack)
                  :
                 );
    // Change the active stack to the process stack
    asm volatile ("msr control, %0;"
                  :
                  : "r"(control)
                  :
                 );
    ibarrier();

    /* Copy initialized data from flash into ram */
    while (dst < &__data_end)
    {
        *dst++ = *src++;
    }

    /* Clear .bss segment in SRAM */
    dst = &__bss_start;
    while (dst < &__bss_end)
    {
        *dst++ = 0;
    }

    // Change the interrupt vector table to that in SRAM so they can be
    // allocated dynamically.
    SCB->vtor = (uint32_t)&sram_isr_vectors;

    ibarrier();

    // We can now call main since the stack, data, and bss are ready!
    main(0, 0);

    while(1);
}

