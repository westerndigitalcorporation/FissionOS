/*
 * sam4_dac.c
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
#include <stdlib.h>

#include <vectors.h>
#include <console.h>

#include "sam4_clock.h"
#include "sam4_gpio.h"
#include "sam4_uart.h"
#include "sam4_dac.h"


static dac_drv_t *dacs_ptr = NULL;

void dac_handle_complete(void)
{
    nvic_disable(PERIPHERAL_ID_DACC);
    DAC->idr = DAC_IDR_EOC;
    (void)DAC->isr;

    if (dacs_ptr->complete)
    {
        dacs_ptr->complete(dacs_ptr->arg);
    }
}

void dac_init(dac_drv_t *dacs)
{
    dacs_ptr = dacs;

    clock_peripheral_start(PERIPHERAL_ID_DACC);
    nvic_callback_set(PERIPHERAL_ID_DACC, dac_handle_complete);

    DAC->cr = DAC_CR_SWRST;
    /* Set the DAC startup to maximum (63) */
    DAC->mr = DAC_MR_STARTUP(63) | DAC_MR_REFRESH(1);
}

int dac_set(uint8_t channel, uint16_t value)
{
    DAC->ier = DAC_IER_EOC;
    nvic_enable(PERIPHERAL_ID_DACC);

    DAC->mr &= ~DAC_MR_USER_SEL_CHANNEL1;
    if (channel)
    {
        DAC->mr |= DAC_MR_USER_SEL_CHANNEL1;
    }

    DAC->cdr = (value & 0xffff);

    return 0;
}

void dac_enable(uint8_t channel)
{
    DAC->cher = (1 << channel);
}

void dac_disable(uint8_t channel)
{
    DAC->chdr = (1 << channel);
}

int cmd_dac(console_t *console, char argc, char *argv[])
{
    uint16_t channel, value;
    if (argc != 3)
    {
        console_print(console, "Not enough arguments\r\n");
        return 0;
    }

    channel = strtoul(argv[1], NULL, 0);
    if (channel >= DAC_CHANNELS)
    {
        console_print(console, "Channel must be 0 or 1\r\n");
        return 0;
    }

    value = strtoul(argv[2], NULL, 0);

    dac_set(channel, value);

    return 0;
}


