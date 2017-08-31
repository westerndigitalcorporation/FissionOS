/*
 * saml_xmodem.c
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

#include "saml_sercom.h"
#include "saml_nvm.h"

#include <systick.h>
#include <console.h>
#include <workqueue.h>
#include <xmodem.h>

#include "saml21_xplained_pro_bootloader.h"
#include "saml_xmodem.h"


void (*console_default_cb)(void*);
void *console_default_arg;
xmodem_t xmodem_handle;
uint32_t xmodem_offset;


uint64_t xmodem_system_time_ms(void)
{
    return ticks * (1000 / SYSTICK_FREQ);
}

uint32_t xmodem_send(void *arg, uint8_t *data, uint8_t len)
{
    uart_drv_t *uart = (uart_drv_t *)xmodem_handle.arg;

    uart_send_wait(uart, (char *)data, len);

    return len;
}

void xmodem_recv_handler(void *arg)
{
    uart_drv_t *uart = (uart_drv_t *)xmodem_handle.arg;
    uint8_t data;

    while (uart_recv(uart, &data, sizeof(data)))
    {
        if (xmodem_recv(&xmodem_handle, &data, sizeof(data)))
        {
            bootcfg_t bootcfg =
            {
                .magic = BOOTCFG_MAGIC,
                .len = xmodem_offset - BOOTLOADER_SIZE,
            };

            // Write out the boot configuration information
            nvm_write(BOOTCFG_ADDR, (uint8_t *)&bootcfg, sizeof(bootcfg));

            console_work.arg = console_default_arg;
            console_work.callback = console_default_cb;

            workqueue_add(&console_work, 0);

            // Save the config word

            return;
        }
    }
}

void xmodem_timer_handler(void *arg);
workqueue_t xmodem_work =
{
    .callback = xmodem_timer_handler,
    .arg = &xmodem_handle,
};

void xmodem_timer_handler(void *arg)
{
    if (xmodem_timer(&xmodem_handle))
    {
        console_work.arg = console_default_arg;
        console_work.callback = console_default_cb;

        return;
    }

    workqueue_add(&xmodem_work, 1);
}

void xmodem_recv_cb(xmodem_t *xmodem, void *arg, uint8_t *data, uint8_t len)
{
    if (!data)
    {
        console_prompt();

        return;
    }

    nvm_write(xmodem_offset, data, len);
    xmodem_offset += len;
}

void xmodem_bootcfg_clear(void)
{
    bootcfg_t bootcfg =
    {
        .magic = 0,
        .len = 0,
    };

    nvm_write(BOOTCFG_ADDR, (uint8_t *)&bootcfg, sizeof(bootcfg));
}

int cmd_xmodem(uart_drv_t *uart, int argc, char *argv[])
{
    if (argc < 2)
    {
        cmd_help_usage(uart, argv[0]);

        return 0;
    }

    if (!strcmp(argv[1], "flash"))
    {
        // Take control of the UART directly
        console_default_cb = console_work.callback;
        console_default_arg = console_work.arg;

        console_work.callback = xmodem_recv_handler;
        console_work.arg = &xmodem_handle;

        console_print("Receiving XModem...\r\n");

        xmodem_offset = BOOTLOADER_SIZE;

        xmodem_init(&xmodem_handle, uart, xmodem_recv_cb);
        xmodem_start(&xmodem_handle);
        xmodem_timer_handler(NULL);

        return 0;
    }
    else if (!strcmp(argv[1], "clear"))
    {
        xmodem_bootcfg_clear();
        return 0;
    }

    cmd_help_usage(uart, argv[0]);

    return 0;
}

