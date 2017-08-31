/*
 * sam4_clock.h
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


#define EXTERNAL_CLK_FREQ        12000000UL
#define INTERNAL_CLK_FREQ        120000000UL

typedef struct pmc
{
    uint32_t scer;
    uint32_t scdr;
    uint32_t scsr;
    uint32_t resvd_0xc;
    uint32_t pcer0;
    uint32_t pcdr0;
    uint32_t pcsr0;
    uint32_t resvd_0x1c;
    uint32_t mor;
#define PMC_MOR_MOSCXTEN        (1 << 0)
#define PMC_MOR_MOSCXTBY        (1 << 1)
#define PMC_MOR_WAITMODE        (1 << 2)
#define PMC_MOR_MOSCRCEN        (1 << 3)
#define PMC_MOR_MOSCRCF(val)    ((val & 0x7) << 4)
#define PMC_MOR_MOSCXTST(val)   ((val & 0xff) << 8)
#define PMC_MOR_KEY             (0x37 << 16)
#define PMC_MOR_MOSCSEL         (1 << 24)
#define PMC_MOR_CFDEN           (1 << 25)
    uint32_t mcfr;
    uint32_t pllar;
#define PMC_PLLAR_DIVA(val)     ((val & 0xff) << 0)
#define PMC_PLLAR_PLLACOUNT(val) ((val & 0x3f) << 8)
#define PMC_PLLAR_MULA(val)     ((val & 0x7ff) << 16)
#define PMC_PLLAR_ONE           (1 << 29)
    uint32_t pllbr;
    uint32_t mckr;
#define PMC_MCKR_CSS_CLK_MASK   (0x3 << 0)
#define PMC_MCKR_CSS_SLOW_CLK   (0x0 << 0)
#define PMC_MCKR_CSS_MAIN_CLK   (0x1 << 0)
#define PMC_MCKR_CSS_PLLA_CLK   (0x2 << 0)
#define PMC_MCKR_CSS_PLLB_CLK   (0x3 << 0)
#define PMC_MCKR_PRES_CLK_MASK  (0x7 << 4)
#define PMC_MCKR_PRES_CLK_1     (0x0 << 4)
#define PMC_MCKR_PRES_CLK_2     (0x1 << 4)
#define PMC_MCKR_PRES_CLK_4     (0x2 << 4)
#define PMC_MCKR_PRES_CLK_8     (0x3 << 4)
#define PMC_MCKR_PRES_CLK_16    (0x4 << 4)
#define PMC_MCKR_PRES_CLK_32    (0x5 << 4)
#define PMC_MCKR_PRES_CLK_64    (0x6 << 4)
#define PMC_MCKR_PRES_CLK_3     (0x7 << 4)
#define PMC_MCKR_PLLADIV2       (1 << 12)
#define PMC_MCKR_PLLBDIV2       (1 << 13)
    uint32_t resvd_0x34;
    uint32_t usb;
    uint32_t resvd_0x3c;
    uint32_t pck0;
    uint32_t pck1;
    uint32_t pck2;
    uint32_t resvd_0x4c[5];
    uint32_t ier;
    uint32_t idr;
    uint32_t sr;
#define PMC_SR_MOSCXTS          (1 << 0)
#define PMC_SR_LOCKA            (1 << 1)
#define PMC_SR_LOCKB            (1 << 2)
#define PMC_SR_MCKRDY           (1 << 3)
#define PMC_SR_OSCSELS          (1 << 7)
#define PMC_SR_PCKRDY0          (1 << 8)
#define PMC_SR_PCKRDY1          (1 << 9)
#define PMC_SR_PCKRDY2          (1 << 10)
#define PMC_SR_MOSCSELS         (1 << 16)
#define PMC_SR_MOSCRCS          (1 << 17)
#define PMC_SR_CFDEV            (1 << 18)
#define PMC_SR_CFDS             (1 << 19)
#define PMC_SR_FOS              (1 << 20)
    uint32_t imr;
    uint32_t fsmr;
    uint32_t fspr;
    uint32_t focr;
    uint32_t resvd_0x7c[26];
    uint32_t wpmr;
    uint32_t wpsr;
    uint32_t resvd_0xec[5];
    uint32_t pcer1;
    uint32_t pcdr1;
    uint32_t pcsr1;
    uint32_t resvd_0x10c;
    uint32_t ocr;
} __attribute__ ((packed)) pmc_t;
#define PMC                     ((volatile pmc_t *)0x400e0400)

int clock_init(void);
void clock_peripheral_start(int peripheral_id);
void clock_peripheral_stop(int peripheral_id);

