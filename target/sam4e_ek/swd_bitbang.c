/*
 * swd_bitbang.c
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

#include "swd.h"
#include "swd_bitbang.h"


/*
 * SWD line protocol gotchas:
 *
 * The SWD line protocol requires that the _least_ significant bit be sent first.
 * This applies to both 8 bit requests, 3 bit acks, and 32 + 1 bit parity.
 * Additionally, the byte order is little endian.
 *
 * Turn around periods between requset and ack are 1.5 clock lengths.  Periods
 * between ack and host write is .5 clock lengths.  Data is clocked out to the
 * slave on high to low edges.  Data clocked out from the slave is done on low
 * to high.
 *
 */

/*
 * This is the general bitbang implementation, to be used by several platforms.
 * Because each platform has different port/pin manipulation operations, a lower
 * layer platform bitbang driver is required.
 *
 * Several functions need to be implemented by the bitbang driver layer.  These
 * are expected to be resolved by the linker.  The required functions are as
 * follows:
 *
 *   void swd_bitbang_clk_dir(struct swd *instance, int output);
 *   void swd_bitbang_clk_set(struct swd *instance, int high);
 *   void swd_bitbang_dio_dir(struct swd *instance, int output);
 *   void swd_bitbang_dio_set(struct swd *instance, int high);
 *   int swd_bitbang_dio_get(struct swd *instance);
 *   void swd_bitbang_udelay(uint32_t usecs);
 */


#define SWD_DELAY_uS(val)                        timer_delay(val)
#define SWD_HTOLE32(val)                         val
#define SWD_LE32TOH(val)                         val

#define SWD_CLK_DELAY_uS                         0          /* As fast as possible, ~400Khz at full clock */

void swd_bitbang_udelay(uint32_t usecs);
#define swd_udelay(val)                          swd_bitbang_udelay(val)


/**
 * Serialize and transmit 8-bits of provided data.
 *
 * @param     instance   SWD Driver Instance Pointer.
 * @param     data       8-bit Data to Transmit.
 *
 * @returns   0 for success, otherwise failure.
 */
static int swd_bitbang_uint8_xmit(swd_t *instance, uint8_t data)
{
    int i;

    for (i = 0; i < 8; i++)
    {
        swd_bitbang_clk_set(instance, 0);
        swd_bitbang_dio_set(instance, (data >> i) & 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);

        swd_bitbang_clk_set(instance, 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
    }

    return 0;
}

/**
 * Serialize and receive 3-bits of ACK data from target.
 *
 * @param     instance   SWD Driver Instance Pointer.
 * @param     ack        8-bit Data Pointer to Receive.
 *
 * @returns   0 for success, -SWD_ERROR_ACK on failure.
 */
static int swd_bitbang_ack_recv(swd_t *instance, uint8_t *ack)
{
    int i;

    *ack = 0;

    swd_bitbang_dio_dir(instance, 0);
    swd_bitbang_clk_set(instance, 0);
    swd_bitbang_udelay(SWD_CLK_DELAY_uS);

    swd_bitbang_clk_set(instance, 1);
    swd_bitbang_udelay(SWD_CLK_DELAY_uS);

    for (i = 0; i < 3; i++)
    {
        swd_bitbang_clk_set(instance, 0);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);

        *ack |= (swd_bitbang_dio_get(instance) << i);

        swd_bitbang_clk_set(instance, 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
    }

    if (*ack != SWD_REQUEST_ACK_ACK)
    {
        return -SWD_ERROR_ACK;
    }

    return 0;
}

/**
 * Serialize and receive 32-bits of response data from target.
 *
 * @param     instance   SWD Driver Instance Pointer.
 * @param     data       32-bit Data Pointer to Receive.
 *
 * @returns   0 for success, negative error on failure.
 */
static int swd_bitbang_uint32_recv(swd_t *instance, uint32_t *data)
{
    uint32_t tmp = 0;
    uint32_t parity = 0;
    uint32_t recv_parity;
    int i;

    for (i = 0; i < 32; i++)
    {
        swd_bitbang_clk_set(instance, 0);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);

        tmp |= (swd_bitbang_dio_get(instance) << i);
        parity ^= swd_bitbang_dio_get(instance);

        swd_bitbang_clk_set(instance, 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
    }

    swd_bitbang_clk_set(instance, 0);
    swd_bitbang_udelay(SWD_CLK_DELAY_uS);

    recv_parity = swd_bitbang_dio_get(instance);

    swd_bitbang_dio_dir(instance, 1);

    *data = SWD_LE32TOH(tmp);

    if (recv_parity != parity)
    {
        return -SWD_ERROR_PARITY;
    }

    return 0;
}

/**
 * Serialize and transmit 32-bits of data to target.
 *
 * @param     instance   SWD Driver Instance Pointer.
 * @param     data       32-bit Data Pointer to Send.
 *
 * @returns   0 for success, negative error on failure.
 */
static int swd_bitbang_uint32_xmit(swd_t *instance, uint32_t data)
{
    uint32_t tmp = SWD_HTOLE32(data);
    uint32_t parity = 0;
    int i;

    swd_bitbang_dio_dir(instance, 1);
    swd_bitbang_dio_set(instance, 1);
    swd_bitbang_clk_set(instance, 0);
    swd_bitbang_udelay(SWD_CLK_DELAY_uS);

    swd_bitbang_clk_set(instance, 1);
    swd_bitbang_udelay(SWD_CLK_DELAY_uS);

    for (i = 0; i < 32; i++)
    {
        swd_bitbang_clk_set(instance, 0);
        swd_bitbang_dio_set(instance, (tmp >> i) & 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);

        swd_bitbang_clk_set(instance, 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);

        parity ^= (tmp >> i) & 1;
    }

    swd_bitbang_clk_set(instance, 0);
    swd_bitbang_dio_set(instance, parity);
    swd_bitbang_udelay(SWD_CLK_DELAY_uS);

    swd_bitbang_clk_set(instance, 1);
    swd_bitbang_udelay(SWD_CLK_DELAY_uS);

    for (i = 0; i < 8; i++)
    {
        swd_bitbang_clk_set(instance, 0);
        swd_bitbang_dio_set(instance, 0);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
        swd_bitbang_clk_set(instance, 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
    }

    return 0;
}


/*
 * The following functions are hooked into the swd core API through the callbacks
 */

/**
 * Compose and transmit a 32-bit data write request.
 *
 * @param     instance   SWD Driver Instance Pointer.
 * @param     request    SDW Request Type.
 * @param     ack        ACK Response From Target.
 * @param     data       32-bit Data to Send.
 *
 * @returns   0 for success, negative error on failure.
 */
int swd_bitbang_request_xmit(swd_t *instance, uint8_t request, uint8_t *ack, uint32_t data)
{
    int result;

    swd_bitbang_uint8_xmit(instance, request);
    result = swd_bitbang_ack_recv(instance, ack);
    if (result)
    {
        return result;
    }

    swd_bitbang_uint32_xmit(instance, data);

    return 0;
}

/**
 * Compose and request a 32-bit data read request.
 *
 * @param     instance   SWD Driver Instance Pointer.
 * @param     request    SDW Request Type.
 * @param     ack        ACK Response From Target.
 * @param     data       32-bit Data Pointer.
 *
 * @returns   0 for success, negative error on failure.
 */
int swd_bitbang_request_recv(swd_t *instance, uint8_t request, uint8_t *ack, uint32_t *data)
{
    int result;

    swd_bitbang_uint8_xmit(instance, request);
    result = swd_bitbang_ack_recv(instance, ack);
    if (result)
    {
        return result;
    }

    result = swd_bitbang_uint32_recv(instance, data);

    return result;
}

/**
 * Force a SWD flush of the write pipeline.
 *
 * @param     instance   SWD Driver Instance Pointer.
 *
 * @returns   0 for success, negative error on failure.
 */
void swd_bitbang_flush(swd_t *instance)
{
    int i;

    swd_bitbang_dio_set(instance, 0);
    for (i = 0; i < 8; i++)
    {
        swd_bitbang_clk_set(instance, 0);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
        swd_bitbang_clk_set(instance, 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
    }
    swd_bitbang_dio_set(instance, 1);
}

/**
 * Send the SWD startup sequence.
 *
 * @param     instance   SWD Driver Instance Pointer.
 *
 * @returns   0
 */
int swd_bitbang_start(swd_t *instance)
{
    uint8_t init_sequence[] = SWD_INIT_SEQUENCE;
    int i;

    // Set SWCLK/SWDIO high output
    swd_bitbang_clk_set(instance, 1);
    swd_bitbang_clk_dir(instance, 1);
    swd_bitbang_dio_set(instance, 1);
    swd_bitbang_dio_dir(instance, 1);

    swd_bitbang_udelay(SWD_CLK_DELAY_uS);

    for (i = 0; i < SWD_INIT_CLK_CYCLES; i++)
    {
        swd_bitbang_clk_set(instance, 0);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
        swd_bitbang_clk_set(instance, 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
    }

    for (i = 0; i < sizeof(init_sequence); i++)
    {
        swd_bitbang_uint8_xmit(instance, init_sequence[i]);
    }

    for (i = 0; i < SWD_INIT_CLK_CYCLES; i++)
    {
        swd_bitbang_clk_set(instance, 0);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
        swd_bitbang_clk_set(instance, 1);
        swd_bitbang_udelay(SWD_CLK_DELAY_uS);
    }

    return 0;
}

/**
 * Cleanup the bitbang driver pin states.
 *
 * @param     instance   SWD Driver Instance Pointer.
 *
 * @returns   0
 */
int swd_bitbang_stop(swd_t *instance)
{
    // Set SWCLK/SWDIO input
    swd_bitbang_clk_dir(instance, 0);
    swd_bitbang_dio_dir(instance, 0);

    return 0;
}

