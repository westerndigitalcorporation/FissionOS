/*
 * saml_clocks.c
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

#include "saml_clocks.h"


#ifdef __AT91SAML21__

void gclk_setup(uint8_t clknum, uint8_t src, uint16_t div)
{
    volatile gclk_t *gclk = GCLK;

    gclk->genctrl[clknum] = GCLK_GENCTRL_GENEN |
                            GCLK_GENCTRL_OE |
                            GCLK_GENCTRL_DIV(div) |
                            src;
}

void gclk_peripheral_enable(uint8_t clknum, uint8_t peripheral)
{
    volatile gclk_t *gclk = GCLK;

    gclk->pchctrl[peripheral] = GCLK_PCHCTRL_GEN(clknum) | GCLK_PCHCTRL_CHEN;
}

void gclk_peripheral_disable(uint8_t clknum, uint8_t peripheral)
{
    volatile gclk_t *gclk = GCLK;

    gclk->pchctrl[peripheral] &= ~GCLK_PCHCTRL_CHEN;
}

#endif /* __AT91SAML21__ */


#ifdef __AT91SAMD20__

void gclk_setup(uint8_t clknum, uint8_t src, uint16_t div)
{
    volatile uint32_t tmp;  // Use temp for unoptimized compiler

    tmp = GCLK_GENDIV_DIV(div) | GCLK_GENDIV_ID(clknum);
    GCLK->gendiv = tmp;
    while (GCLK->status)
        ;

    tmp = GCLK_GENCTRL_ID(clknum) |
          GCLK_GENCTRL_SRC(src) |
          GCLK_GENCTRL_OE |
          GCLK_GENCTRL_GENEN;
    GCLK->genctrl = tmp;
    while (GCLK->status)
        ;
}

void gclk_peripheral_enable(uint8_t clknum, uint8_t peripheral)
{
    volatile uint16_t tmp;  // Use temp for unoptimized compiles

    tmp = GCLK_CLKCTRL_ID(peripheral) | GCLK_CLKCTRL_GEN(clknum) | GCLK_CLKCTRL_CLKEN;
    GCLK->clkctrl = tmp;
    while (GCLK->status)
        ;
}

void gclk_peripheral_disable(uint8_t clknum, uint8_t peripheral)
{
    volatile uint16_t tmp;

    *(uint8_t *)&GCLK->clkctrl = peripheral;

    tmp = GCLK->clkctrl;
    tmp &= ~GCLK_CLKCTRL_CLKEN;

    GCLK->clkctrl = tmp;
}

#endif /* __AT91SAMD20__ */
