/*
 * sam4_ac.c
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

#include "vectors.h"

#include "sam4_clock.h"
#include "sam4_gpio.h"
#include "sam4_ac.h"

static ac_callback_t ac_callback = NULL;
static void *ac_arg = NULL;

static void ac_handle_interrupt(void)
{
    uint32_t isr = ACC->isr;

    // Throw out interrupts detected during hardware initialization
    if ((isr & ACC_ISR_MASK) || !(isr & ACC_ISR_CE))
    {
        return;
    }

    if (ac_callback)
    {
        ac_callback((isr & ACC_ISR_SCO) ? 1 : 0, ac_arg);
    }
}

void ac_init(ac_callback_t callback, void *arg, uint32_t mr)
{
    ac_callback = callback;
    ac_arg = arg;

    clock_peripheral_start(PERIPHERAL_ID_ACC);
    nvic_callback_set(PERIPHERAL_ID_ACC, ac_handle_interrupt);

    ACC->cr = ACC_CR_SWRST;
    ACC->acr = ACC_ACR_ISEL | ACC_ACR_HYST(0x3);
    ACC->mr = mr;
}

void ac_enable(void)
{
    ACC->ier = ACC_IER_CE;
    nvic_enable(PERIPHERAL_ID_ACC);

    ACC->mr |= ACC_MR_ACEN;

    ac_handle_interrupt();
}

void ac_disable(void)
{
    ACC->mr &= ~ACC_MR_ACEN;

    nvic_disable(PERIPHERAL_ID_ACC);
    ACC->idr = ACC_IER_CE;
}

