/*
 * usb_serial.c
 *
 *
 * Copyright (c) 2017 Jeremy Garff
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
 * Author: Jeremy Garff <jer@jers.net>
 *
 */

#include <stdio.h>
#include <string.h>

#include <vectors.h>
#include <console.h>
#include <systick.h>

#include "usb.h"

#include "usb_serial.h"


#define USB_XMIT_STATE_IDLE                      0
#define USB_XMIT_STATE_BUSY                      1

#define USB_SERIAL_MAX_XFER_BYTES                64
#define USB_SERIAL_MAX_INT_BYTES                 8

// Wait 20mS
#define USB_SERIAL_TIMEOUT_TICKS                 ((SYSTICK_FREQ * 200) / 1000)

usb_serial_rx_cb_t usb_serial_rx_cb;
void *usb_serial_rx_arg = NULL;

static usb_endpoint_entry_t usb_serial_data_out;
void usb_serial_recv_notify(usb_endpoint_entry_t *ep)
{
    usb_serial_rx_cb(ep, usb_serial_rx_arg);
}

int usb_serial_recv(usb_endpoint_entry_t *ep, char *buffer, int maxlen)
{
    uint8_t len = usb_endpoint_buffer_read(ep, buffer, maxlen);

    return len;
}

volatile uint8_t usb_serial_xmit_state = USB_XMIT_STATE_IDLE;
static void usb_serial_xmit(usb_endpoint_entry_t *ep)
{
    int result = usb_txbuffer_done(ep);

    if (!result)
    {
        usb_serial_xmit_state = USB_XMIT_STATE_IDLE;
    }
}

static void usb_serial_wakeup(usb_endpoint_entry_t *ep)
{
    usb_endpoint_rxout_enable(ep);
}

static uint8_t usb_serial_int_buf[USB_SERIAL_MAX_INT_BYTES];
static usb_endpoint_entry_t usb_serial_int =
{
    .num        = 1,
    .type       = USB_ENDPOINT_TYPE_BULK,
    .direction  = USB_ENDPOINT_DIR_IN,
    .size       = USB_SERIAL_MAX_INT_BYTES,
    .banks      = USB_ENDPOINT_BANKS_1,
    .recv_buf   = usb_serial_int_buf,

    .rx_setup   = NULL,
    .rx_out     = NULL,
    .tx_in      = NULL,
    .sleep      = NULL,
    .wakeup     = usb_serial_wakeup,
    .reset      = usb_serial_wakeup,
};

static uint8_t usb_serial_rxbuf[USB_SERIAL_MAX_XFER_BYTES];
static usb_endpoint_entry_t usb_serial_data_out = 
{
    .num        = 2,
    .type       = USB_ENDPOINT_TYPE_BULK,
    .direction  = USB_ENDPOINT_DIR_OUT,
    .size       = USB_SERIAL_MAX_XFER_BYTES,
    .banks      = USB_ENDPOINT_BANKS_1,
    .recv_buf   = usb_serial_rxbuf,

    .rx_setup   = NULL, 
    .rx_out     = usb_serial_recv_notify,
    .tx_in      = usb_serial_xmit,
    .sleep      = NULL,
    .wakeup     = usb_serial_wakeup,
    .reset      = usb_serial_wakeup,
};

int usb_serial_send(char *buffer, int len)
{
    int irqstate = irq_save();
    int result;

    if (usb_serial_xmit_state != USB_XMIT_STATE_IDLE)
    {
        irq_restore(irqstate);
        return 0;
    }

    usb_serial_xmit_state = USB_XMIT_STATE_BUSY;
    result = usb_txbuffer_start(&usb_serial_data_out, buffer, len);

    irq_restore(irqstate);

    return result;
}

int usb_serial_send_wait(char *buffer, int len)
{
    uint32_t timeout_ticks = ticks + USB_SERIAL_TIMEOUT_TICKS;
    int sent = 0;

    while (sent < len)
    {
        int result = usb_serial_send(buffer, len);

        while (usb_serial_xmit_state != USB_XMIT_STATE_IDLE)
        {
            if (timeout_ticks <= ticks)
            {
                return sent;
            }
        }

        sent += result;

        buffer += sent;
        len -= sent;

    }

    return sent;
}

void usb_console_send_wait(void *dev, char *data, uint32_t len)
{
    usb_serial_send_wait(data, len);
}

int usb_console_recv(void *dev, char *data, uint32_t maxlen)
{
    return usb_endpoint_buffer_read(&usb_serial_data_out, data, maxlen);
}

void usb_console_rx_callback(usb_endpoint_entry_t *ep, void *arg)
{
    console_t *console = (console_t *)arg;
    console_rx_schedule(console);
}

int usb_serial_init(usb_serial_rx_cb_t rx_callback, void *arg)
{
    usb_serial_rx_cb = rx_callback;
    usb_serial_rx_arg = arg;

    usb_endpoint_register(&usb_serial_int);
    usb_endpoint_register(&usb_serial_data_out);

    return 0;
}

