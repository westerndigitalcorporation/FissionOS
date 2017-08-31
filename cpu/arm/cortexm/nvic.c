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


#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "systick.h"

#include "vectors.h"


void console_print(char *format, ...);


extern uint32_t __text_start, __data_end, __etext, __bss_start,
                __bss_end, __main_stack, __exception_stack;


uint32_t irq_save(void)
{
    uint32_t primask;
    uint32_t set = 0x1;

    asm volatile ("mrs %0, primask;"
                  : "=r"(primask)
                  :
                  :
                 );
    asm volatile ("msr primask, %0;"
                  :
                  : "r"(set)
                  :
                 );

    return primask;
}

void irq_restore(uint32_t primask)
{
    asm volatile ("msr primask, %0;"
                  :
                  : "r"(primask)
                  :
                 );
}


void reset_handler(void);
void thread_switch_handler(void);   /**< From context.c */

void console_printreg(char *str, uint32_t val)
{
    console_print("%s%08x  ", str, (unsigned int)val);
}

void exception_stack_display(char *regname, uint32_t sp)
{
    console_printreg(regname, sp);
    console_print("\r\n");
    console_printreg("R0:   ", ((uint32_t *)sp)[0]);
    console_printreg("R1:   ", ((uint32_t *)sp)[1]);
    console_printreg("R2:   ", ((uint32_t *)sp)[2]);
    console_printreg("R3:   ", ((uint32_t *)sp)[3]);
    console_print("\r\n");
    console_printreg("R12:  ", ((uint32_t *)sp)[4]);
    console_printreg("LR:   ", ((uint32_t *)sp)[5] & ~1);
    console_printreg("PC:   ", ((uint32_t *)sp)[6]);
    console_printreg("xPSR: ", ((uint32_t *)sp)[7]);
    console_print("\r\n\n");
}

void exception_handler(char *str)
{
    uint32_t sp, psr, control, lr;

    console_print("\r\n");
    console_print(str);
    console_print("\r\n");

    // Get the process stack pointer
    asm volatile ("mrs %0, psp;"
                  : "=r"(sp)        /* output register %0 */
                  :                 /* no input */
                  :                 /* no clobbered register */
            );
    exception_stack_display("PSP:  ", sp);

    // Get the exception/interrupt stack pointer
    asm volatile ("mrs %0, msp;"
                  : "=r"(sp)        /* output register %0 */
                  :                 /* no input */
                  :                 /* no clobbered register */
            );
    exception_stack_display("MSP:  ", sp);

    console_printreg("SCB_CFSR:  ", SCB->cfsr);
    console_printreg("SCB_HFSR:  ", SCB->hfsr);
    console_print("\r\n");
    console_printreg("SCB_MMFAR: ", SCB->mmfar);
    console_printreg("SCB_AFSR:  ", SCB->afsr);
    console_print("\r\n\n");

    // This will contain the interrupt
    asm volatile ("mrs %0, psr;"
                  : "=r"(psr)
                  :
                  :
                 );

    console_printreg("PSR: ", psr);
    console_printreg("INT: ", psr & 0x1ff);
    console_print("\r\n\n");

    asm volatile ("mrs %0, control;"
                  : "=r"(control)
                  :
                  :
                 );
    console_printreg("CONTROL: ", control);
    asm volatile ("mov %0, lr;"
                  : "=r"(lr)
                  :
                  :
                 );
    console_printreg("LR: ", lr);
    console_print("\r\n\n");
    console_print("\r\n\n");

    while(1)
        ;
}

void hardfault_handler(void)
{
    exception_handler("Hard Fault");
}

void mmufault_handler(void)
{
    exception_handler("MPU Fault");
}

void busfault_handler(void)
{
    exception_handler("BUS Fault");
}

void usagefault_handler(void)
{
    exception_handler("Usage Fault");
}

void dbg_handler(void)
{
    exception_handler("Debug Fault");
}

void interrupt_unhandled(void)
{
    exception_handler("Unhandled Interrupt\r\n");
}

// Flash interrupt vector table, only used for bootstrap
__attribute__ ((section(".flash_isr_vector")))
struct vector_table flash_isr_vectors =
{
    .sp = &__exception_stack,
    .reset_handler = reset_handler,
    .nmi_handler = interrupt_unhandled,
    .hardfault_handler = hardfault_handler,
    .mmufault_handler = mmufault_handler,
    .busfault_handler = busfault_handler,
    .usagefault_handler = usagefault_handler,
    .dbg_handler = dbg_handler,
};

// SRAM interrupt vector table, used by main application
__attribute__ ((section(".sram_isr_vector")))
struct vector_table sram_isr_vectors =
{
    .sp = &__exception_stack,
    .reset_handler = reset_handler,
    .nmi_handler = interrupt_unhandled,
    .hardfault_handler = hardfault_handler,
    .mmufault_handler = mmufault_handler,
    .busfault_handler = busfault_handler,
    .usagefault_handler = usagefault_handler,
    .dbg_handler = dbg_handler,
    .svcall_handler = thread_switch_handler,
    .systick_handler = systick_handler,
};

// Interrupt manipulation functions
void nvic_callback_set(int isr, isr_handler callback)
{
    // The hardware will call the pointer assigned in this table directly.
    sram_isr_vectors.vector[isr] = callback;
}

void nvic_enable(int isr)
{
    NVIC->iser[isr >> 5] = 1 << (isr & 0x1f);
}

void nvic_disable(int isr)
{
    NVIC->icer[isr >> 5] = 1 << (isr & 0x1f);
}

void nvic_clear_pending(int isr)
{
    NVIC->icpr[isr >> 5] = 1 << (isr & 0x1f);
}

uint32_t nvic_isr(void)
{
    uint32_t ipsr;

    asm volatile ("mrs %0, ipsr;"
                  : "=r"(ipsr)
                  :
                  :
                 );

    return ipsr;
}
