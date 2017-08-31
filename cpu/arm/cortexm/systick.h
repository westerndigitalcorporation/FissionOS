/*
 * sam4_systick.h
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


#ifndef __SYSTICK_H__
#define __SYSTICK_H__


#define SYSTICK_FREQ             60


typedef struct syst
{
    uint32_t csr;
#define SYST_CSR_ENABLE          (1 << 0)
#define SYST_CSR_TICKINT         (1 << 1)
#define SYST_CSR_CLKSOURCE       (1 << 2)
#define SYST_CSR_COUNTFLAG       (1 << 16)
    uint32_t rvr;
#define SYST_RVR_RELOAD(val)     ((val & 0xffffff) << 0)
    uint32_t cvr;
#define SYST_CVR_CURRENT(val)    ((val & 0xffffff) << 0)
    uint32_t calib;
} __attribute__ ((packed)) syst_t;

#define SYST                     ((volatile syst_t *)0xe000e010)

void systick_handler(void);
void systick_init(uint32_t clk_freq);
void udelay(uint32_t usecs);

extern volatile uint32_t ticks;

#endif /* __SYSTICK_H__ */
