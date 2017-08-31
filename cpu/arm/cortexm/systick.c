/*
 * sam4_systick.c
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
#include <string.h>

#include "vectors.h"

#include "systick.h"

static uint32_t internal_clk_freq;

volatile uint32_t ticks;
void systick_handler(void)
{
    ticks++;
}

void systick_init(uint32_t clk_freq)
{
    internal_clk_freq = clk_freq;

    SYST->csr = SYST_CSR_COUNTFLAG | SYST_CSR_CLKSOURCE;
    barrier();

    SYST->rvr = (internal_clk_freq / SYSTICK_FREQ) - 1;
    barrier();

    SYST->csr = SYST_CSR_ENABLE | SYST_CSR_TICKINT |
                SYST_CSR_COUNTFLAG | SYST_CSR_CLKSOURCE;
}

void udelay(uint32_t usecs)
{
    uint32_t systicks = (SYSTICK_FREQ * usecs) / 1000000UL;
    uint32_t remainder = usecs % (1000000UL / SYSTICK_FREQ);
    uint32_t expire_ticks = ticks + systicks;
    uint32_t expire_cvr = SYST->cvr;

    if ((expire_cvr - remainder) > expire_cvr)
    {
        uint32_t tmp = remainder - expire_cvr;

        expire_cvr = (internal_clk_freq / SYSTICK_FREQ) - tmp - 1;
    }
    else
    {
        expire_cvr -= remainder;
    }

    while (ticks < expire_ticks)
        ;

    while (SYST->cvr > expire_cvr);
}

