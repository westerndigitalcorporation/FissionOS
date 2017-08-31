/*
 * xmodem.c
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
#include <stdio.h>

#include "xmodem.h"


#define XMODEM_SOH                               0x01
#define XMODEM_EOT                               0x04
#define XMODEM_ACK                               0x06
#define XMODEM_NACK                              0x15
#define XMODEM_CAN                               0x18

#define XMODEM_TIMEOUT_START                     1000   // mS
#define XMODEM_TIMEOUT_NACK                      3000  // mS
#define XMODEM_NACK_RETRIES                      30
#define XMODEM_START_RETRIES                     60


static uint8_t xmodem_sendbuf;                   /**< Global ACK/NACK buffer */


/*
 * Internal functions
 */

/**
 * Send XModem ACK packet and update timer expirations, packet counts, and
 * buffer offsets appropriately.
 *
 * @param    xmodem  XModem context.
 *
 * @returns  None
 */
static void xmodem_ack(xmodem_t *xmodem)
{
    xmodem->timeout = xmodem_system_time_ms() + XMODEM_TIMEOUT_START;
    xmodem->len = 0;
    xmodem->packet_num++;
    xmodem->nak_count = 0;

    xmodem_sendbuf = XMODEM_ACK;
    xmodem_send(xmodem->arg, &xmodem_sendbuf, sizeof(xmodem_sendbuf));
}

/**
 * Update counters and lengths appropriately for NACK processing.  Let the
 * timer expiration routes actually send the NAK itself.
 *
 * @param    xmodem  XModem context.
 *
 * @returns  None
 */
static void xmodem_nak(xmodem_t *xmodem)
{
    xmodem->len = 0;
    xmodem->nak_count++;
}

/**
 * Given a array of bytes, compute the checksum and return it.
 *
 * @param   data  Pointer to byte array on which to compute checksum
 * @param   len   Length in bytes of the data array.
 *
 * @returns 8-bit checksum value
 */
static uint8_t xmodem_cksum(const uint8_t *data, const uint8_t len)
{
    uint8_t cksum = 0, i;

    for (i = 0; i < len; i++)
    {
        cksum += data[i];
    }

    return cksum;
}


/*
 * API Functions
 */

/**
 * API function to initialize the XModem state machine and callbacks.
 *
 * @param    xmodem   The xmodem instance pointer.
 * @param    arg      Private context for the caller.
 * @param    recv_cb  Receiver callback to issue when receiving valid
 *                    data.
 *
 * @returns  None
 */
void xmodem_init(xmodem_t *xmodem, void *arg, xmodem_recv_cb_t recv_cb)
{
    xmodem->state = XMODEM_STATE_IDLE;
    xmodem->arg = arg;
    xmodem->recv_cb = recv_cb;
}

/**
 * API function to reset the XModem context and begin a XModem transfer.
 *
 * @param    xmodem   The xmodem instance pointer.
 *
 * @returns  None
 */
void xmodem_start(xmodem_t *xmodem)
{
    xmodem->state = XMODEM_STATE_START;
    xmodem->timeout = xmodem_system_time_ms() + XMODEM_TIMEOUT_START;
    xmodem->packet_num = 1;
    xmodem->nak_count = 0;
    xmodem->len = 0;
}

/**
 * API function to process XModem timer state changes.  Must be called by
 * the OS at a minimum of 1 second intervals.  The xmodem_send() and
 * xmodem_system_time_ms() functions need to be implemented by a outside
 * OS portability layer.
 *
 * @param    xmodem   The xmodem instance pointer.
 *
 * @returns  None
 */
int xmodem_timer(xmodem_t *xmodem)
{
    switch (xmodem->state)
    {
        case XMODEM_STATE_IDLE:
            // Do nothing
            break;

        case XMODEM_STATE_START:
            if (xmodem_system_time_ms() >= xmodem->timeout)
            {
                xmodem_nak(xmodem);

                xmodem_sendbuf = XMODEM_NACK;
                xmodem_send(xmodem->arg, &xmodem_sendbuf, sizeof(xmodem_sendbuf));

                xmodem->timeout = xmodem_system_time_ms() + XMODEM_TIMEOUT_START;
                if (xmodem->nak_count >= XMODEM_START_RETRIES)
                {
                    goto done;
                }
            }
            break;

        default:
            // Check for NACK timeout
            if (xmodem_system_time_ms() >= xmodem->timeout)
            {
                xmodem_nak(xmodem);

                xmodem_sendbuf = XMODEM_NACK;
                xmodem_send(xmodem->arg, &xmodem_sendbuf, sizeof(xmodem_sendbuf));

                xmodem->timeout = xmodem_system_time_ms() + XMODEM_TIMEOUT_NACK;
                if (xmodem->nak_count >= XMODEM_NACK_RETRIES)
                {
                    goto done;
                }
            }
            break;
    }

    return 0;

done:
    xmodem->state = XMODEM_STATE_IDLE;

    if (xmodem->recv_cb)
    {
        xmodem->recv_cb(xmodem, xmodem->arg, NULL, 0);
    }

    return 1;
}

/**
 * API function to process received bytes.  Must be called by the OS.
 * The xmodem_send() and xmodem_system_time_ms() functions need
 * to be implemented by a outside OS portability layer.
 *
 * @param    xmodem   The xmodem instance pointer.
 *
 * @returns  None
 */
int xmodem_recv(xmodem_t *xmodem, uint8_t *data, uint8_t len)
{
    while (len--)
    {
        uint8_t cksum;

        ((uint8_t *)&xmodem->packet)[xmodem->len++] = *data;

        // Check for end of transfer
        if ((xmodem->len == 1) && (*data == XMODEM_EOT))
        {
            xmodem->state = XMODEM_STATE_IDLE;
            xmodem_ack(xmodem);

            // Issue the callback with 0 length data to indicate completion
            if (xmodem->recv_cb)
            {
                xmodem->recv_cb(xmodem, xmodem->arg, NULL, 0);
            }

            return 1;
        }

        // Got a full packet?
        if (xmodem->len >= sizeof(xmodem->packet))
        {
            //
            // Verify the packet contents
            //
            if (xmodem->packet.soh != XMODEM_SOH)
            {
                xmodem_nak(xmodem);
                continue;
            }

            if ((xmodem->packet_num != xmodem->packet.num) ||
                (xmodem->packet_num != (~xmodem->packet.inv_num & 0xff)))
            {
                xmodem_nak(xmodem);
                continue;
            }

            cksum = xmodem_cksum((uint8_t *)&xmodem->packet, sizeof(xmodem->packet) - sizeof(cksum));
            if (cksum != xmodem->packet.cksum)
            {
                xmodem_nak(xmodem);
                continue;
            }

            //
            // It's good, we'll take it
            //
            if (xmodem->recv_cb)
            {
                xmodem->recv_cb(xmodem, xmodem->arg, xmodem->packet.data, sizeof(xmodem->packet.data));
            }

            // ACK the packet
            xmodem->state = XMODEM_STATE_SOH;
            xmodem_ack(xmodem);
        }
    }

    return 0;
}


