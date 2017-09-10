/*
 * sam4_tc.c
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
#if defined(__AT91SAM4S__)


#include <stdint.h>
#include <stddef.h>

#include <vectors.h>
#include <console.h>


#include "sam4_gpio.h"
#include "sam4_clock.h"
#include "sam4_uart.h"
#include "sam4_tc.h"


#define SAM4_TC_FREQ_CLKDIV                      32


static tc_drv_t *tcs[TC_MAX];


int tc_channel_to_index(tc_drv_t *dev, int channel)
{
    int index;

    if (channel >= SAM4_MAX_CHANNELS)
    {
        return -1;
    }

    if (dev->tc == TC0)
    {
        index = channel;
    }
    else if (dev->tc == TC1)
    {
        index = channel + SAM4_MAX_CHANNELS;
    }
    else
    {
        return -1;
    }

    return index;
}

static void tc_channel_handler(tc_drv_t *drv)
{
    volatile tc_channel_t *channel = &drv->tc->channel[drv->channel];
    uint32_t status = channel->sr;

    channel->idr = TC_CHANNEL_IDR_LDRBS | TC_CHANNEL_IDR_COVFS;
    channel->ccr = TC_CHANNEL_CCR_CLKDIS;

    if (drv->handler)
    {
        uint32_t ticks_sec = INTERNAL_CLK_FREQ / SAM4_TC_FREQ_CLKDIV;
        uint32_t freq = ticks_sec / channel->rb;

        if (status & TC_CHANNEL_SR_COVFS)
        {
            freq = 0;
        }

        drv->handler(drv, freq);
    }
}

static void tc_channel0_handler(void)
{
    tc_channel_handler(tcs[0]);
}

static void tc_channel1_handler(void)
{
    tc_channel_handler(tcs[1]);
}

static void tc_channel2_handler(void)
{
    tc_channel_handler(tcs[2]);
}

static void tc_channel3_handler(void)
{
    tc_channel_handler(tcs[3]);
}

static void tc_channel4_handler(void)
{
    tc_channel_handler(tcs[4]);
}

static void tc_channel5_handler(void)
{
    tc_channel_handler(tcs[5]);
}

int tc_init(tc_drv_t *dev)
{
    void (*vector)(void) = NULL;
    int peripheral_id, index;

    index = tc_channel_to_index(dev, dev->channel);
    if (index < 0)
    {
        return -1;
    }

    switch (index)
    {
        case 0:
            vector = tc_channel0_handler;
            break;
        case 1:
            vector = tc_channel1_handler;
            break;
        case 2:
            vector = tc_channel2_handler;
            break;
        case 3:
            vector = tc_channel3_handler;
            break;
        case 4:
            vector = tc_channel4_handler;
            break;
        case 5:
            vector = tc_channel5_handler;
            break;
    }

    peripheral_id = PERIPHERAL_ID_TC0 + index;
    tcs[index] = dev;

    dev->tc->channel[dev->channel].ccr = TC_CHANNEL_CCR_CLKDIS;           // disable

    clock_peripheral_start(peripheral_id);
    nvic_callback_set(peripheral_id, vector);
    nvic_enable(peripheral_id);

    return 0;
}

int tc_capture_freq(tc_drv_t *dev)
{
    volatile tc_channel_t *channel = &dev->tc->channel[dev->channel];

    channel->ccr = TC_CHANNEL_CCR_CLKDIS;           // disable
    channel->ra  = 0;
    channel->rb  = 0;
    channel->rc  = 0;                               // Clear the clock counter
    channel->cmr = TC_CHANNEL_TCCLKS_TIMER_CLOCK3 | // clk / 32
                   TC_CHANNEL_ETRGEDG_RISING      | // start on rising edge
                   TC_CHANNEL_ABETRG              | // channel a used for trigger
                   TC_CHANNEL_LDBDIS              | // disable on load b
                   TC_CHANNEL_LDRA_FALLING        | // load a on falling edge
                   TC_CHANNEL_LDRB_RISING;          // load b on rising edge
    channel->ier = TC_CHANNEL_IER_LDRBS |           // load b triggers interrupt
                   TC_CHANNEL_IER_COVFS;            //   or overflow
    channel->ccr = TC_CHANNEL_CCR_CLKEN;            // enable

    return 0;
}

int cmd_tc(console_t *console, int argc, char *argv[])
{
    int i;

    for (i = 0; i < (sizeof(tcs) / sizeof(tcs[0])); i++)
    {
        if (tcs[i])
        {
            volatile tc_channel_t *channel = &tcs[i]->tc->channel[tcs[i]->channel];
            console_print(console, "%2d %2d CCR %08x CMR %08x SR %08x CV %08x RA %08x RB %08x\r\n", i, tcs[i]->channel,
                          channel->ccr, channel->cmr, channel->sr, channel->cv, channel->ra, channel->rb);
        }
    }

    return 0;
}

#endif

