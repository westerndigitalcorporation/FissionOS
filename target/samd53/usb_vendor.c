/*
 * hwheader.h
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


#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "usb.h"
#include "usb_messages.h"
#include "usb_config.h"
#include "product.h"
#include "workqueue.h"
#include "fwheader.h"
#include "systick.h"

#include <saml_nvm.h>
#include <saml_reset.h>

#include "usb_vendor.h"


#define FLASH_STATE_NONE                         0
#define FLASH_STATE_APP                          1
#define FLASH_STATE_SIG                          2

#define WORKER_TYPE_NONE                         0
#define WORKER_TYPE_RESET                        1
#define WORKER_TYPE_SWITCH_BANK                  2

uint32_t page_offset = 0;
uint32_t page_addr;
uint32_t flash_state = FLASH_STATE_NONE;

device_info_t device_info;


void usb_vendor_worker(void *arg)
{
    uint32_t type = (uint32_t)arg;

    switch (type)
    {
        case WORKER_TYPE_RESET:
            saml_soft_reset();
            break;

        case WORKER_TYPE_SWITCH_BANK:
            nvm_switch_bank();
            break;

        default:
            break;
    }

    return;
}

workqueue_t switch_bank_wq =
{
    .callback = usb_vendor_worker,
    .arg = NULL,
};

static void rx_vendor_setup(usb_endpoint_entry_t *ep, usb_request_t *req)
{
    switch (req->request)
    {
        case USB_VENDOR_REQUEST_RESET:
            usb_txbuffer_start(ep, NULL, 0);
            // Schedule the bank switch in 1 second to make sure
            // the USB request is completed first
            switch_bank_wq.arg = (void *)WORKER_TYPE_RESET;
            workqueue_add(&switch_bank_wq, SYSTICK_FREQ);

            break;

        case USB_VENDOR_REQUEST_INFO:
            usb_txbuffer_start(ep, (char *)&device_info, sizeof(device_info));

            break;

        case USB_VENDOR_REQUEST_FLASH:
            nvm_cache_disable();

            page_addr = (((uint32_t)req->value[0] << 24) |
                        ((uint32_t)req->value[1] << 16) |
                        (uint32_t)req->index);
            page_offset = 0;

            flash_state = FLASH_STATE_APP;

            break;

        case USB_VENDOR_REQUEST_FLASH_DONE:
            flash_state = FLASH_STATE_SIG;

            break;
    }
}

static uint16_t rx_vendor_out(usb_endpoint_entry_t *ep)
{
    uint32_t addr = page_addr + page_offset;
	int len;
    char buf[64];

    len = usb_endpoint_buffer_read(ep, buf, sizeof(buf));

    switch (flash_state) {
        case FLASH_STATE_APP:
            if (len >= 0)
            {
                nvm_write(addr, (uint8_t *)buf, len);
                page_offset += len;
            }

            break;

        case FLASH_STATE_SIG:
            if (len == sizeof(fwheader_t))
            {
                fwheader_t *header = (fwheader_t *)buf;
                uint32_t crc;

                // Validate the header a bit
                if (header->magic != FWHEADER_MAGIC)
                {
                    break;
                }

                // Make sure last page is written
                nvm_write(addr, NULL, 0);

                nvm_cache_enable();

                crc = nvm_crc32(0, header->len);
                if (crc != header->crc)
                {
                    break;
                }

                // Schedule the bank switch in 1 second to make sure
                // the USB request is completed first
                switch_bank_wq.arg = (void *)WORKER_TYPE_SWITCH_BANK;
                workqueue_add(&switch_bank_wq, SYSTICK_FREQ);
            }

            break;

        default:
            // Do nothing
            break;
    }

    return len;
}

void usb_vendor_init(void)
{
    device_info.bank = nvm_active_bank();
    device_info.size = nvm_bank_offset();

    // Register callbacks to handle vendor specific traffic.
    usb_control0_vendor_register(rx_vendor_setup, rx_vendor_out);
}


