/*
 * saml_tccc
 *
 *
 * Copyright (c) 2017 Western Digital Corporation or its affiliates.
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

#include <console.h>

#include "saml_tcc.h"


void tcc_pwm_init(volatile tcc_t *tcc, uint32_t prescaler,
                 uint8_t invertmask, uint16_t period)
{
    tcc->ctrla &= ~TCC_CTRLA_ENABLE;
    while (tcc->syncbusy)
        ;

    tcc->ctrla = TCC_CTRLA_SWRST;
    while (tcc->syncbusy)
        ;

    tcc->ctrla = TCC_CTRLA_PRESCALER(prescaler) | TCC_CTRLA_PRESYNC_PRESC;
    tcc->drvctrl = TCC_DRVCTRL_INVMASK(invertmask);
    tcc->wave = TCC_WAVE_WAVEGEN_NPWM;
    tcc->dbgctrl = TCC_DBGCTRL_DBGRUN;

    tcc->per = period;

    while (tcc->syncbusy)
        ;

    tcc->ctrla |= TCC_CTRLA_ENABLE; // Enable
}

void tcc_disable(volatile tcc_t *tcc)
{
    tcc->ctrla &= ~TCC_CTRLA_ENABLE;
}

void tcc_pwm_duty(volatile tcc_t *tcc, int channel, uint16_t duty)
{
    tcc->cc[channel] = duty;
}


