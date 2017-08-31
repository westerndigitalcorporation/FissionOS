/*
 * console.c
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


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>

#if !defined(__AT91SAML21__) && !defined(__AT91SAMD20__)
#include "sam4_gpio.h"
#include "sam4_uart.h"
#include "sam4_watchdog.h"
#include "sam4_clock.h"
#include "sam4_flash.h"
#include "sam4_vectors.h"
#else /* __AT91SAML21__ */
#include "saml_sercom.h"
#endif /* __AT91SAML21__ */

#include "console.h"
#include "workqueue.h"


void console_rx_callback(uart_drv_t *uart, void *arg);

char bs_str[] = { 0x08, ' ', 0x08, 0 };  // backspace overwrite
char *crlf = "\r\n";
char *cmd_prompt = "> ";


static cmdline_t console_cmdline;
static cmd_entry_t *cmd_table;
static int cmd_table_len = 0;

#define CONSOLE_PRINT_SIZE  120
void console_print(char *format, ...)
{
    char buf[CONSOLE_PRINT_SIZE];
    va_list ap;
    int n;

    va_start(ap, format);
    n = vsnprintf(buf, CONSOLE_PRINT_SIZE, format, ap);
    va_end(ap);

    uart_send_wait(console_cmdline.uart, buf, n);
}

#if !defined(__AT91SAML21__) && !defined(__AT91SAMD20__)
void console_init(uart_drv_t *uart, cmd_entry_t *table, int table_len,
                  volatile gpio_regs_t *tx_port, int tx_pin,
                  volatile gpio_regs_t *rx_port, int rx_pin)
#else /* __AT91SAML21__ */
void console_init(uart_drv_t *uart, cmd_entry_t *table, int table_len)
#endif /* __AT91SAML21__ */
{
    cmd_table = table;
    cmd_table_len = table_len;

    uart->rx_cb = console_rx_callback;
    uart->rx_arg = &console_cmdline;
    console_cmdline.uart = uart;

#if !defined(__AT91SAML21__) && !defined(__AT91SAMD20__)
    GPIO_DISABLE(tx_port, tx_pin);
    GPIO_PERIPHERAL_SET(tx_port, tx_pin, 0);

    GPIO_DISABLE(rx_port, rx_pin);
    GPIO_PERIPHERAL_SET(rx_port, rx_pin, 0);
#endif /* __AT91SAML21__ */

    uart_init(uart);
}

void console_prompt(void)
{
    console_print("%s", cmd_prompt);
}

int cmd_help_usage(uart_drv_t *uart, char *command)
{
    int i;

    for (i = 0; i < cmd_table_len; i++) {
        if (!strcmp(command, cmd_table[i].cmdstr)) {
            if (cmd_table[i].usage) {
                console_print("\r\nUsage:\r\n");
                uart_send_wait(uart, cmd_table[i].usage, strlen(cmd_table[i].usage));
                console_print("\r\n");
            }
            if (cmd_table[i].help) {
                console_print("Description:\r\n");
                uart_send_wait(uart, cmd_table[i].help, strlen(cmd_table[i].help));
                console_print("\r\n");
            }

            return 0;
        }
    }

    console_print("Invalid command\r\n");

    return 0;
}

int cmd_help(uart_drv_t *uart, int argc, char *argv[])
{
    char *cmd_list = "\nCommand list:\r\n\n";
    int i;

    if (argc > 1) {
        cmd_help_usage(uart, argv[1]);
    } else {
        // Command list
        uart_send_wait(uart, cmd_list, strlen(cmd_list));
        for (i = 0; i < cmd_table_len; i++) {
            uart_send_wait(uart, cmd_table[i].cmdstr, strlen(cmd_table[i].cmdstr));
            uart_send_wait(uart, crlf, strlen(crlf));
        }
        uart_send_wait(uart, crlf, strlen(crlf));
    }

    return 0;
}

int cmd_handle(uart_drv_t *uart, int argc, char *argv[])
{
    char *unknown_cmd = "\rUnknown command\r\n";
    int i;

    for (i = 0; i < cmd_table_len; i++)
    {
        if (!strcmp(argv[0], cmd_table[i].cmdstr))
        {
            return cmd_table[i].callback(uart, argc, argv);
        }
    }

    uart_send_wait(uart, unknown_cmd, strlen(unknown_cmd));

    return 0;
}

int console_doescape(cmdline_t *cmdline, char c)
{
    switch (cmdline->state)
    {
        case CMDLINE_STATE_NOESCAPE:
            if (c == 0x1b)
            {
                cmdline->state = CMDLINE_STATE_ESCAPE;
                return 1;
            }
            break;

        case CMDLINE_STATE_ESCAPE:
            if (c == '[')
            {
                cmdline->state = CMDLINE_STATE_CMD;
                return 1;
            }
            cmdline->state = CMDLINE_STATE_NOESCAPE;
            break;

        case CMDLINE_STATE_CMD:
            if ((c >= '0') && (c <= '9'))
            {
                return 1;
            }

            if (c == 'A')
            {
                // Handle up arrow
                while (cmdline->offset)
                {
                    uart_send_wait(cmdline->uart, bs_str, sizeof(bs_str));
                    cmdline->offset--;
                }

                strncpy(cmdline->buffer, cmdline->prev, CMDLINE_SIZE);
                uart_send_wait(cmdline->uart, cmdline->buffer, strlen(cmdline->buffer));
                cmdline->offset = strlen(cmdline->buffer);
            }
            cmdline->state = CMDLINE_STATE_NOESCAPE;
            return 1;
    }

    return 0;
}

int console_cmdparse(cmdline_t *cmdline)
{
    char *args_max = "Too many arguments\r\n";
    char *argv[MAX_ARGS] = { NULL };
    char last = ' ';
    int i = 0, argc = 0, len = strlen(cmdline->buffer);
    int result = 0;

    if (len)
    {
        strncpy(cmdline->prev, cmdline->buffer, CMDLINE_SIZE);
    }

    do
    {
        if (!cmdline->buffer[i])
        {
            break;
        }

        if ((last == ' ') && (cmdline->buffer[i] != ' '))
        {
            if (argc >= MAX_ARGS)
            {
                uart_send_wait(cmdline->uart, args_max, strlen(args_max));
                goto done;
            }

            argv[argc++] = &cmdline->buffer[i];
        }

        last = cmdline->buffer[i];
        if (cmdline->buffer[i] == ' ')
        {
            cmdline->buffer[i] = 0;
        }

        i++;
    } while (i < len);

    if (argc)
    {
        result = cmd_handle(cmdline->uart, argc, argv);
    }

done:
    cmdline->buffer[0] = 0;
    cmdline->offset = 0;
    uart_send_wait(cmdline->uart, cmd_prompt, strlen(cmd_prompt));

    return result;
}

int console_getcmdline(cmdline_t *cmdline, char *newdata, uint32_t len)
{
    while (cmdline->offset < sizeof(cmdline->buffer) - 1)
    {
        char c;

        if (!len)
        {
            return -1;
        }

        c = *newdata++;
        len--;

        if (console_doescape(cmdline, c))
        {
            continue;
        }

        // Check for valid characters
        if (((c >= 'a') && (c <= 'z')) ||
            ((c >= 'A') && (c <= 'Z')) ||
            ((c >= '0') && (c <= '9')) ||
            (c == 0x7f) || (c == 0x08) ||
            (c == 0x0d) || (c == 0x0a) ||
            (c == ' ') || (c == '-'))
        {
            // Echo all but del/bs/lf/cr
            if ((c != 0x0d) && (c != 0x0a) &&
                (c != 0x7f) && (c != 0x08))
            {
                uart_send_wait(cmdline->uart, &c, sizeof(c));
            }
        }
        else
        {
            continue;
        }

        // Deal with known chars
        switch (c)
        {
            case 0x7f:
            case 0x08:  // backspace
                if (!cmdline->offset)
                {
                    continue;
                }
                cmdline->offset--;
                uart_send_wait(cmdline->uart, bs_str, sizeof(bs_str));
                continue;

            case 0xd:
            case 0xa:   // linefeed
                cmdline->buffer[cmdline->offset] = 0;
                goto done;

            default:
                cmdline->buffer[cmdline->offset] = c;
                break;
        }

        cmdline->offset++;
    }

done:
    cmdline->buffer[cmdline->offset] = 0;
    uart_send_wait(cmdline->uart, crlf, strlen(crlf));

    return 0;
}

void console_work_handler(void *arg);
workqueue_t console_work =
{
    .callback = console_work_handler,
    .arg = &console_cmdline,
};

void console_work_handler(void *arg)
{
    cmdline_t *cmdline = (cmdline_t *)arg;
    uint8_t data;

    while (uart_recv(cmdline->uart, &data, sizeof(data)))
    {
        if (console_getcmdline(cmdline, (char *)&data, sizeof(data)))
        {
            continue;
        }

        console_cmdparse(cmdline);
    }
}

void console_rx_callback(uart_drv_t *uart, void *arg)
{
    workqueue_add(&console_work, 0);
}

