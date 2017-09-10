/*
 * usb_control0.c
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

#if defined(__AT91SAML21__) || defined(__ATSAMD53__)

#include <stdio.h>
#include <string.h>

#include "usb.h"



#define CTRL_STATE_IDLE                          0 // Nada going on
#define CTRL_STATE_ADDR                          1 // We have a set address request that must
                                                   // be effective only after the transaction
#define CTRL_STATE_VENDOR_OUT                    2 // Vendor out packet(s) following a setup

static usb_desc_string_t **usb_str_desc = NULL;
static char *usb_desc = NULL;
static uint32_t usb_desc_len;
static char *usb_config = NULL;
static uint32_t usb_config_len;

static volatile uint8_t ctrl0_state = CTRL_STATE_IDLE;  // Global state flags
static uint16_t vendor_out_expected = 0;

static rx_vendor_setup_t *control0_vendor_setup = NULL;
static rx_vendor_out_t *control0_vendor_out = NULL;

static char usb_status[2] = { 0, 0 }; // Always return 0 since we are always bus powered.


void control0_handle_desc(usb_endpoint_entry_t *ep, usb_request_t *req)
{
    switch (req->value[1])
    {
        case USB_DESC_DEVICE:
            usb_txbuffer_start(ep, usb_desc, usb_desc_len);
            break;

        case USB_DESC_CONFIG:
            usb_txbuffer_start(ep, usb_config, 
                               usb_config_len > req->length ?
                               req->length : usb_config_len);
            break;

        case USB_DESC_STRING:
            usb_txbuffer_start(ep, (char *)usb_str_desc[req->value[0]],
                               usb_str_desc[req->value[0]]->length > req->length ?
                               req->length : usb_str_desc[req->value[0]]->length);
            break;
    }
}

void control0_handle_vendor(usb_endpoint_entry_t *ep, usb_request_t *req)
{
    if (control0_vendor_setup)
    {
        control0_vendor_setup(ep, req);
    }

    if (!(req->request_type & USB_REQ_DIR_DEV_TO_HOST))
    {
        vendor_out_expected = req->length;
        ctrl0_state = CTRL_STATE_VENDOR_OUT;
    }
}

void control0_rx_setup(usb_endpoint_entry_t *ep)
{
    usb_request_t req;
    uint8_t len;

    if ((len = usb_endpoint_buffer_read(ep, (char *)&req, sizeof(req))) != sizeof(req))
    {
        return;
    }

    if ((req.request_type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_VENDOR)
    {
        control0_handle_vendor(ep, &req);
        return;
    }

    switch (req.request)
    {
        case USB_REQ_GET_DESCRIPTOR:
            control0_handle_desc(ep, &req);
            break;

        case USB_REQ_ADDRESS:
            usb_address_set(req.value[0]);
            ctrl0_state = CTRL_STATE_ADDR;
            usb_txbuffer_start(ep, NULL, 0);
            break;

        case USB_REQ_SET_INTERFACE:
        case USB_REQ_SET_CONFIG:
            usb_txbuffer_start(ep, NULL, 0);
            break;

        case USB_REQ_GET_STATUS:
            usb_txbuffer_start(ep, usb_status, sizeof(usb_status));
            break;

        // CDC ACM state/coding
        case USB_REQ_SET_CONTROL_LINE_STATE:
            usb_txbuffer_start(ep, NULL, 0);
            break;

        case USB_REQ_SET_LINE_CODING:
            usb_txbuffer_start(ep, NULL, 0);
            break;
    }
}

void control0_buffer_continue(usb_endpoint_entry_t *ep)
{
    int result = usb_txbuffer_done(ep);

    if (!result && (ctrl0_state == CTRL_STATE_ADDR))
    {
        // We are done responding to the set address request,
        // so now we can enable the new address for further packets.
        ctrl0_state = CTRL_STATE_IDLE;
        usb_address_enable();
    }
}

void control0_rx_out(usb_endpoint_entry_t *ep)
{
    char buf[64];

    if ((ctrl0_state == CTRL_STATE_VENDOR_OUT) && control0_vendor_out)
    {
        vendor_out_expected -= control0_vendor_out(ep);
    }
    else
    {
        int len = sizeof(buf) < vendor_out_expected ? sizeof(buf) : vendor_out_expected;
        vendor_out_expected -= usb_endpoint_buffer_read(ep, buf, len);
    }

    if (!vendor_out_expected)
    {
        ctrl0_state = CTRL_STATE_IDLE;
        usb_txbuffer_start(ep, NULL, 0);
    }
}

void control0_wakeup(usb_endpoint_entry_t *ep)
{
    // On wakeup, enable the enpoint interrupts we care about
    usb_endpoint_rxstp_enable(ep);
    usb_endpoint_rxout_enable(ep);
}


// Default control endpoint entry
uint8_t usb_control0_rxbuf[64];
usb_endpoint_entry_t usb_control0_endpoint = {
    .num       = 0,
    .type      = USB_ENDPOINT_TYPE_CONTROL,
    .direction = USB_ENDPOINT_DIR_OUT,
    .size      = sizeof(usb_control0_rxbuf), // Full speed control devices must be 64 
    .recv_buf  = usb_control0_rxbuf,

    // Interrupt handlers
    .rx_setup  = control0_rx_setup,
    .tx_in     = control0_buffer_continue,
    .rx_out    = control0_rx_out,  
    .sleep     = NULL,
    .wakeup    = control0_wakeup,
    .reset     = control0_wakeup
};

void usb_control0_vendor_register(rx_vendor_setup_t *rx_vendor_setup,
                                  rx_vendor_out_t *rx_vendor_out)
{
    control0_vendor_setup = rx_vendor_setup;
    control0_vendor_out = rx_vendor_out;
}

int usb_control0_init(char *desc, uint32_t desc_len,
                      char *config, uint32_t config_len,
                      usb_desc_string_t **str_desc)
{
    usb_str_desc = str_desc;
    usb_desc = desc;
    usb_desc_len = desc_len;
    usb_config = config;
    usb_config_len = config_len;

    usb_endpoint_register(&usb_control0_endpoint);

    return 0;
}

#endif /* __AT91SAML21__ */
