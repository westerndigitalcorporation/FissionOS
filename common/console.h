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

struct console;
typedef int (*cmd_callback_t)(struct console *console, int argc, char *argv[]);
typedef struct cmd_entry
{
    char *cmdstr;
    char *usage;
    char *help;
    cmd_callback_t callback;
} cmd_entry_t;

typedef int (*console_send_t)(void *arg, char *buf, int buflen);
typedef int (*console_recv_t)(void *arg, char *buf, int buflen);
typedef struct console
{
    cmd_entry_t *cmd_table;
    int cmd_table_len;
    char prev[CMDLINE_SIZE];
    char buffer[CMDLINE_SIZE];
    int offset;
    int state;
#define CMDLINE_STATE_NOESCAPE      0
#define CMDLINE_STATE_ESCAPE        1
#define CMDLINE_STATE_CMD           2
    workqueue_t wq;
    console_send_t send;
    console_recv_t recv;
    void *arg;
} console_t;

void console_prompt(console_t *console);
void console_print(console_t *console, char *format, ...);
int cmd_help(console_t *console, int argc, char *argv[]);
int cmd_help_usage(console_t *console, char *command);
void console_rx_schedule(console_t *console);

void console_init(console_t *console, cmd_entry_t *table, int table_len,
                  console_send_t send, console_recv_t recv, void *arg);

extern void console_work_handler(void *arg);

#endif
