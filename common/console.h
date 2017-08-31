/*
 * console.h
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

#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#if !defined(__AT91SAML21__) && !defined(__AT91SAMD20__)
#include "sam4_gpio.h"
#endif /* __AT91SAML21__ */


#include "workqueue.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array)          (sizeof(array) / sizeof(array[0]))
#endif

#define CMDLINE_SIZE               80
#define MAX_ARGS                   16

#define CONSOLE_CMD_HELP                         \
    {                                            \
        .cmdstr = "help",                        \
        .callback = cmd_help,                    \
        .usage = "  help [ command ]\r\n",       \
        .help =                                  \
            "  List commands or provide help on specified command.\r\n" \
            "    command : Command help to display\r\n", \
    }

typedef int (*cmd_callback_t)(uart_drv_t *uart, int argc, char *argv[]);
typedef struct cmd_entry
{
    char *cmdstr;
    char *usage;
    char *help;
    cmd_callback_t callback;
} cmd_entry_t;

typedef struct cmdline
{
    uart_drv_t *uart;
    char prev[CMDLINE_SIZE];
    char buffer[CMDLINE_SIZE];
    int offset;
    int state;
#define CMDLINE_STATE_NOESCAPE      0
#define CMDLINE_STATE_ESCAPE        1
#define CMDLINE_STATE_CMD           2
} cmdline_t;

void console_prompt(void);
void console_print(char *format, ...);
int cmd_help(uart_drv_t *uart, int argc, char *argv[]);
int cmd_help_usage(uart_drv_t *uart, char *command);

#if !defined(__AT91SAML21__) && !defined(__AT91SAMD20__)
void console_init(uart_drv_t *uart, cmd_entry_t *table, int table_len,
                  volatile gpio_regs_t *tx_port, int tx_pin,
                  volatile gpio_regs_t *rx_port, int rx_pin);
#else /* __AT91SAML21__ */
// TODO:  Make this the new more portable interface for the 4S
void console_init(uart_drv_t *uart, cmd_entry_t *table, int table_len);
#endif /* __AT91SAML21__ */

// The following are only required if a command issued from the console needs to take
// direct control (xmodem).
extern workqueue_t console_work;
extern void console_work_handler(void *arg);

#endif
