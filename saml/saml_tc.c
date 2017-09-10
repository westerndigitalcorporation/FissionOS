/*
 * saml_tc.c
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

#include <console.h>

#include "saml_tc.h"


void tc_pwm_init(volatile tc_t *tc, uint32_t prescale_flag,
                 uint8_t invert)
{
    tc->ctrla = TC_CTRLA_SWRST;
    while (tc->syncbusy)
        ;

    tc->ctrla = TC_CTRLA_MODE_COUNT16 | prescale_flag;
    tc->drvctrl = invert ? TC_DRVCTRL_INVEN0 : 0;
    tc->wave = TC_WAVE_NPWM;
    tc->dbgctrl = TC_DBGCTRL_DBGRUN;

    while (tc->syncbusy)
        ;

    tc->ctrla |= TC_CTRLA_ENABLE; // Enable
}

void tc_disable(volatile tc_t *tc)
{
    tc->ctrla &= ~TC_CTRLA_ENABLE;
}

void tc_pwm_duty(volatile tc_t *tc, int channel, uint16_t duty)
{
    if (!channel)
    {
        tc->ccbuf0 = duty;
    }
    else
    {
        tc->ccbuf1 = duty;
    }
}

int cmd_tc(console_t *console, int argc, char *argv[])
{
    volatile tc_t *tc = TC0;

    if ((argc == 2) && !strcmp(argv[1], "show"))
    {
        console_print(console, "Main Registers\r\n");

        tc->ctrlbset = TC_CTRLBSET_CMD_READSYNC;
        while (tc->syncbusy)
            ;
        console_print(console, "  count   : 0x%08x\r\n", tc->count);
    }
    else
    {
        cmd_help_usage(console, argv[0]);
    }

    return 0;
}

