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

#include "console.h"
#include "workqueue.h"


void console_rx_callback(console_t *console, void *arg);

char bs_str[] = { 0x08, ' ', 0x08, 0 };  // backspace overwrite
char *crlf = "\r\n";
char *cmd_prompt = "> ";


#define CONSOLE_PRINT_SIZE  120
void console_print(console_t *console, char *format, ...)
{
    char buf[CONSOLE_PRINT_SIZE];
    va_list ap;
    int n;

    va_start(ap, format);
    n = vsnprintf(buf, CONSOLE_PRINT_SIZE, format, ap);
    va_end(ap);

    console->send(console->arg, buf, n);
}

void console_init(console_t *console, cmd_entry_t *table, int table_len,
                  console_send_t send, console_recv_t recv, void *arg)
{
    console->cmd_table = table;
    console->cmd_table_len = table_len;
    console->send = send;
    console->recv = recv;
    console->arg = arg;
}

void console_prompt(console_t *console)
{
    console_print(console, "%s", cmd_prompt);
}

int cmd_help_usage(console_t *console, char *command)
{
    int i;

    for (i = 0; i < console->cmd_table_len; i++) {
        if (!strcmp(command, console->cmd_table[i].cmdstr)) {
            if (console->cmd_table[i].usage) {
                console_print(console, "\r\nUsage:\r\n");
                console->send(console->arg, console->cmd_table[i].usage, strlen(console->cmd_table[i].usage));
                console_print(console, "\r\n");
            }
            if (console->cmd_table[i].help) {
                console_print(console, "Description:\r\n");
                console->send(console->arg, console->cmd_table[i].help, strlen(console->cmd_table[i].help));
                console_print(console, "\r\n");
            }

            return 0;
        }
    }

    console_print(console, "Invalid command\r\n");

    return 0;
}

int cmd_help(console_t *console, int argc, char *argv[])
{
    char *cmd_list = "\nCommand list:\r\n\n";
    int i;

    if (argc > 1) {
        cmd_help_usage(console, argv[1]);
    } else {
        // Command list
        console->send(console->arg, cmd_list, strlen(cmd_list));
        for (i = 0; i < console->cmd_table_len; i++) {
            console->send(console->arg, console->cmd_table[i].cmdstr, strlen(console->cmd_table[i].cmdstr));
            console->send(console->arg, crlf, strlen(crlf));
        }
        console->send(console->arg, crlf, strlen(crlf));
    }

    return 0;
}

int cmd_handle(console_t *console, int argc, char *argv[])
{
    char *unknown_cmd = "\rUnknown command\r\n";
    int i;

    for (i = 0; i < console->cmd_table_len; i++)
    {
        if (!strcmp(argv[0], console->cmd_table[i].cmdstr))
        {
            return console->cmd_table[i].callback(console, argc, argv);
        }
    }

    console->send(console->arg, unknown_cmd, strlen(unknown_cmd));

    return 0;
}

int console_doescape(console_t *console, char c)
{
    switch (console->state)
    {
        case CMDLINE_STATE_NOESCAPE:
            if (c == 0x1b)
            {
                console->state = CMDLINE_STATE_ESCAPE;
                return 1;
            }
            break;

        case CMDLINE_STATE_ESCAPE:
            if (c == '[')
            {
                console->state = CMDLINE_STATE_CMD;
                return 1;
            }
            console->state = CMDLINE_STATE_NOESCAPE;
            break;

        case CMDLINE_STATE_CMD:
            if ((c >= '0') && (c <= '9'))
            {
                return 1;
            }

            if (c == 'A')
            {
                // Handle up arrow
                while (console->offset)
                {
                    console->send(console->arg, bs_str, sizeof(bs_str));
                    console->offset--;
                }

                strncpy(console->buffer, console->prev, CMDLINE_SIZE);
                console->send(console->arg, console->buffer, strlen(console->buffer));
                console->offset = strlen(console->buffer);
            }
            console->state = CMDLINE_STATE_NOESCAPE;
            return 1;
    }

    return 0;
}

int console_cmdparse(console_t *console)
{
    char *args_max = "Too many arguments\r\n";
    char *argv[MAX_ARGS] = { NULL };
    char last = ' ';
    int i = 0, argc = 0, len = strlen(console->buffer);
    int result = 0;

    if (len)
    {
        strncpy(console->prev, console->buffer, CMDLINE_SIZE);
    }

    do
    {
        if (!console->buffer[i])
        {
            break;
        }

        if ((last == ' ') && (console->buffer[i] != ' '))
        {
            if (argc >= MAX_ARGS)
            {
                console->send(console->arg, args_max, strlen(args_max));
                goto done;
            }

            argv[argc++] = &console->buffer[i];
        }

        last = console->buffer[i];
        if (console->buffer[i] == ' ')
        {
            console->buffer[i] = 0;
        }

        i++;
    } while (i < len);

    if (argc)
    {
        result = cmd_handle(console, argc, argv);
    }

done:
    console->buffer[0] = 0;
    console->offset = 0;
    console->send(console->arg, cmd_prompt, strlen(cmd_prompt));

    return result;
}

int console_getcmdline(console_t *console, char *newdata, uint32_t len)
{
    while (len)
    {
        while (console->offset < sizeof(console->buffer) - 1)
        {
            char c;

            if (!len)
            {
                return -1;
            }

            c = *newdata++;
            len--;

            if (console_doescape(console, c))
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
                    console->send(console->arg, &c, sizeof(c));
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
                    if (!console->offset)
                    {
                        continue;
                    }
                    console->offset--;
                    console->send(console->arg, bs_str, sizeof(bs_str));
                    continue;

                case 0xd:
                case 0xa:   // linefeed
                    console->buffer[console->offset] = 0;
                    goto done;

                default:
                    console->buffer[console->offset] = c;
                    break;
            }

            console->offset++;
        }

        len--;
        newdata++;
    }

done:
    console->buffer[console->offset] = 0;
    console->send(console->arg, crlf, strlen(crlf));

    return 0;
}

void console_work_handler(void *arg)
{
    console_t *console = (console_t *)arg;
    char data[64];
    int n;

    while ((n = console->recv(console->arg, data, sizeof(data))))
    {
        if (console_getcmdline(console, data, n))
        {
            continue;
        }

        console_cmdparse(console);
    }
}

void console_rx_schedule(console_t *console)
{
    console->wq.callback = console_work_handler;
    console->wq.arg = console;
    workqueue_add(&console->wq, 0);
}

