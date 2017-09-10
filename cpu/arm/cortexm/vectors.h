/*
 * k20_vectors.h
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


#ifndef __VECTORS_H__
#define __VECTORS_H__


#if defined(__AT91SAM4S__) || defined(__AT91SAM4E__)
#include "sam4_vectors.h"
#elif defined(__K20__)
#include "k20_vectors.h"
#elif defined(__AT91SAML21__) || defined(__AT91SAMD20__) || defined(__ATSAMD53__)
#include "saml_vectors.h"
#else
#error Unknown platform
#endif


/* Standard ARM Cortex-M core exception numbers */
#define ISR_THREAD               0
#define ISR_NMI                  2
#define ISR_HARD_FAULT           3
#define ISR_MM_FAULT             4
#define ISR_BUS_FAULT            5
#define ISR_USAGE_FAULT          6
#define ISR_SVCALL               11
#define ISR_PENDSV               14
#define ISR_SYSTICK              15
#define ISR_IRQ_BASE             16


#define SCS_BASE_ADDR            0xe000e000
#define SCB_BASE_ADDR            (SCS_BASE_ADDR + 0x0d00)

typedef struct scb
{
    uint32_t cpuid;
    uint32_t icsr;
    uint32_t vtor;
    uint32_t aircr;
    uint32_t scr;
    uint32_t ccr;
#define SCB_CCR_NONBASETHRDENA  (1 << 0)
#define SCB_CCR_USERSETMPEND    (1 << 1)
#define SCB_CCR_UNALIGN_TRP     (1 << 3)
#define SCB_CCR_DIV_0_TRP       (1 << 4)
#define CCB_CCR_BFHFNMIGN       (1 << 8)
#define CCB_CCR_STKALIGN        (1 << 9)
    uint32_t shpr2;
    uint32_t shpr3;
    uint32_t shcsr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t afsr;
} __attribute__ ((packed)) scb_t;
#define SCB                     ((volatile scb_t *)SCB_BASE_ADDR)


typedef struct nvic
{
    uint32_t iser[8];
    uint32_t resvd_0x120[24];
    uint32_t icer[8];
    uint32_t resvd_0x1a0[24];
    uint32_t ispr[8];
    uint32_t resvd_0x220[24];
    uint32_t icpr[8];
    uint32_t resvd_0x2a0[24];
    uint32_t iabr[8];
    uint32_t resvd_0x320[56];
    uint32_t ipr[9];
    uint32_t resvd_0x424[695];
    uint32_t stir;
} __attribute__ ((packed)) nvic_t;
#define NVIC                    ((volatile nvic_t *)0xe000e100)


typedef void (*isr_handler)(void);

struct vector_table
{
    uint32_t *sp;
    isr_handler reset_handler;
    isr_handler nmi_handler;
    isr_handler hardfault_handler;
    isr_handler mmufault_handler;
    isr_handler busfault_handler;
    isr_handler usagefault_handler;
    uint32_t resvd_1c;
    uint32_t resvd_20;
    uint32_t resvd_24;
    uint32_t resvd_28;
    isr_handler svcall_handler;
    isr_handler dbg_handler;
    uint32_t resvd_34;
    isr_handler pensv_handler;
    isr_handler systick_handler;
    isr_handler vector[ISR_MAX];   // Defined in <platform_>vectors.h
} __attribute__ ((packed)) vector_table_t;


static inline void syscall(void)
{
    asm volatile ("svc #0;");
}

static inline void cpu_sleep(void)
{
    asm volatile ("wfi;");
}

static inline void barrier(void)
{
    asm volatile ("dsb;");
}

static inline void ibarrier(void)
{
    asm volatile ("isb;");
}

void nvic_callback_set(int isr, isr_handler callback);
void nvic_enable(int isr);
void nvic_disable(int isr);
void nvic_clear_pending(int isr);
uint32_t nvic_isr(void);

uint32_t irq_save(void);
void irq_restore(uint32_t primask);

void exception_handler(char *str);

#endif /* __VECTORS_H__ */
