/*
 * sam4_ac.h
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


#ifndef __SAM4S_AC_H__
#define __SAM4S_AC_H__

typedef struct acc
{
    uint32_t cr;
#define ACC_CR_SWRST                             (1 << 0)
    uint32_t mr;
#define ACC_MR_SELMINUS_TS                       ((0 & 0x7) << 0)
#define ACC_MR_SELMINUS_ADVREF                   ((1 & 0x7) << 0)
#define ACC_MR_SELMINUS_DAC0                     ((2 & 0x7) << 0)
#define ACC_MR_SELMINUS_DAC1                     ((3 & 0x7) << 0)
#define ACC_MR_SELMINUS_AD0                      ((4 & 0x7) << 0)
#define ACC_MR_SELMINUS_AD1                      ((5 & 0x7) << 0)
#define ACC_MR_SELMINUS_AD2                      ((6 & 0x7) << 0)
#define ACC_MR_SELMINUS_AD3                      ((7 & 0x7) << 0)
#define ACC_MR_SELPLUS_AD0                       ((0 & 0x7) << 4)
#define ACC_MR_SELPLUS_AD1                       ((1 & 0x7) << 4)
#define ACC_MR_SELPLUS_AD2                       ((2 & 0x7) << 4)
#define ACC_MR_SELPLUS_AD3                       ((3 & 0x7) << 4)
#define ACC_MR_SELPLUS_AD4                       ((4 & 0x7) << 4)
#define ACC_MR_SELPLUS_AD5                       ((5 & 0x7) << 4)
#define ACC_MR_SELPLUS_AD6                       ((6 & 0x7) << 4)
#define ACC_MR_SELPLUS_AD7                       ((7 & 0x7) << 4)
#define ACC_MR_ACEN                              (1 << 8)
#define ACC_MR_EDGETYP_RISING                    ((0 & 0x3) << 9)
#define ACC_MR_EDGETYP_FALLING                   ((1 & 0x3) << 9)
#define ACC_MR_EDGETYP_ANY                       ((2 & 0x3) << 9)
#define ACC_MR_INV                               (1 << 12)
#define ACC_MR_SELFS                             (1 << 13)
#define ACC_MR_FE                                (1 << 14)
    uint32_t resvd_0x08[7];
    uint32_t ier;
#define ACC_IER_CE                               (1 << 0)
    uint32_t idr;
#define ACC_IDR_CE                               (1 << 0)
    uint32_t imr;
#define ACC_IMR_CE                               (1 << 0)
    uint32_t isr;
#define ACC_ISR_CE                               (1 << 0)
#define ACC_ISR_SCO                              (1 << 1)
#define ACC_ISR_MASK                             (1 << 31)
    uint32_t resvd_0x34[24];
    uint32_t acr;
#define ACC_ACR_ISEL                             (1 << 0)
#define ACC_ACR_HYST(val)                        ((val & 0x3) << 1)
    uint32_t resvd_0x98[19];
    uint32_t wpmr;
    uint32_t wpsr;
} __attribute__ ((packed)) acc_t;

#if defined(__AT91SAM4S__)
#define ACC                                      ((volatile acc_t *)0x40040000)
#elif defined(__AT91SAM4E__)
#define ACC                                      ((volatile acc_t *)0x400bc000)
#endif


typedef void (*ac_callback_t)(uint32_t over, void *arg);

void ac_init(ac_callback_t callback, void *arg, uint32_t mr);
void ac_enable(void);
void ac_disable(void);

#endif /* __SAM4S_AC_H__ */
