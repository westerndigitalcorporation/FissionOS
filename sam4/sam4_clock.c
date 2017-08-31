/*
 * sam4_clock.c
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
#include <stddef.h>

#include "sam4_clock.h"
#include "sam4_flash.h"

void clock_peripheral_start(int peripheral_id)
{
    if (peripheral_id < (sizeof(uint32_t) * 8))
    {
        PMC->pcer0 = (1 << peripheral_id);
    }
    else
    {
        PMC->pcer1 = (1 << (peripheral_id - (sizeof(uint32_t) * 8)));
    }
}

void clock_peripheral_stop(int peripheral_id)
{
    if (peripheral_id < 32)
    {
        PMC->pcdr0 = (1 << peripheral_id);
    }
    else
    {
        PMC->pcdr1 = (1 << (peripheral_id - (sizeof(uint32_t) * 8)));
    }
}

static int clock_crystal_init(void)
{
    // TODO:  Detect crystal failures and return error code, code will hang in failure
    //        currently.

    // Turn on the crystal oscillator
    PMC->mor = (PMC->mor & ~PMC_MOR_MOSCXTBY) |
               PMC_MOR_MOSCXTEN | PMC_MOR_MOSCXTST(0xff) |
               PMC_MOR_KEY;
    while (!(PMC->sr & PMC_SR_MOSCXTS))
        ;

    PMC->mor |= PMC_MOR_KEY | PMC_MOR_MOSCSEL;

    // The following will configure PLLA to generate 240Mhz from a
    // 12Mhz input clock
    PMC->pllar = PMC_PLLAR_DIVA(1) |
                 PMC_PLLAR_PLLACOUNT(0x3f) |
                 PMC_PLLAR_MULA(19) |         // 120Mhz
                 PMC_PLLAR_ONE;
    while (!(PMC->sr & PMC_SR_LOCKA))
        ;

    // Divide 192Mhz PPLA to generate 96Mhz for master clock
    PMC->mckr = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_MAIN_CLK;
    while (!(PMC->sr & PMC_SR_MCKRDY))
        ;

    // Set the flash to 6 wait states since we're going to 120Mhz, flash
    // sequential, loop optimization and 128-bit accesses are enabled.
    EEFC0->fmr = EEFC_FMR_FWS(6) | EEFC_FMR_CLOE;
    while (!(EEFC0->fsr & EEFC_FSR_FRDY))
        ;

#if defined(__AT91SAM4S__) && defined(__DUAL_BANK_FLASH__)
    // Set the flash to 6 wait states since we're going to 120Mhz, flash
    // sequential, loop optimization and 128-bit accesses are enabled.
    EEFC1->fmr = EEFC_FMR_FWS(6) | EEFC_FMR_CLOE;
    while (!(EEFC1->fsr & EEFC_FSR_FRDY))
        ;
#endif

    // Change master clock to PLLA, CPU clock is slaved off master.
    PMC->mckr = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK;
    while (!(PMC->sr & PMC_SR_MCKRDY))
        ;

    // Turn on the cache controller
#if defined(__AT91SAM4E__) && defined(__CPU_DATA_CACHE__)
    *(uint32_t *)0x400c4008 = 0x1;
#endif

#if defined(__AT91SAM4S__) && defined(__CPU_DATA_CACHE__)
    *(uint32_t *)0x4007c008 = 0x1;
#endif

    // Now at 120Mhz, 6 wait states on 128-bit code optimized flash controller

    return 0;
}

int clock_init(void)
{
    int ret;

    ret = clock_crystal_init();

    return ret;
}

