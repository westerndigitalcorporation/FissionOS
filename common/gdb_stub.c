/*
 * gdb_stub.c
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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <lwip/ip_addr.h>
#include <lwip/netif.h>
#include <lwip/tcpip.h>
#include <lwip/api.h>

#include "swd.h"
#include "swd_target.h"

#include "swd_gdb_backend.h"
#include "gdb_stub.h"


#define GDB_STACK_SIZE                           2048
#define GDB_POLL_mS                              500

typedef struct swd_gdb_client
{
    struct netconn *client;
} swd_gdb_client_t;


extern swd_t *swd_instance;


void swd_gdb_write(swd_gdb_client_t *conn, char *data, uint32_t len)
{
    netconn_write(conn->client, data, len, NETCONN_COPY);
}

int gdb_handle_client(struct netconn *client)
{
    struct netbuf *rx_buf;
    err_t err;

    netconn_set_recvtimeout(client, GDB_POLL_mS);

    err = netconn_recv(client, &rx_buf);
    while ((err == ERR_OK) || (err == ERR_TIMEOUT))
    {
        swd_gdb_client_t conn;
        uint8_t *data;
        uint16_t len;

        conn.client = client;

        gdb_poll(&conn, swd_instance);

        if (err == ERR_OK)
        {
            netbuf_data(rx_buf, (void **)&data, &len);

            gdb_stream_process(&conn, swd_instance, data, len);

            if (netbuf_next(rx_buf) != ERR_OK)
            {
                netbuf_delete(rx_buf);
                err = netconn_recv(client, &rx_buf);
            }
        }
    }

    return 0;
}

void gdb_mainloop(void *arg)
{
    struct netconn *gdb_netconn = (struct netconn *)arg;
    while (1)
    {
        struct netconn *client;
        err_t err;

        err = netconn_accept(gdb_netconn, &client);
        if (err != ERR_OK)
            continue;

        swd_target_halt(swd_instance);
        gdb_handle_client(client);

        netconn_close(client);
        netconn_delete(client);
    }
}

static thread_t gdb_thread;
static uint8_t gdb_stack[GDB_STACK_SIZE];
void gdb_service_start(void)
{
    struct netconn *gdb_netconn;
    err_t err;

	gdb_netconn = netconn_new(NETCONN_TCP);
    if (!gdb_netconn)
    {
        return;
    }

	err = netconn_bind(gdb_netconn, IP_ADDR_ANY, 3333);
    if (err != ERR_OK)
    {
        netconn_delete(gdb_netconn);
        return;
    }

	err = netconn_listen(gdb_netconn);
    if (err != ERR_OK)
    {
        netconn_delete(gdb_netconn);
        return;
    }

    thread_new(&gdb_thread, gdb_stack, sizeof(gdb_stack), gdb_mainloop, gdb_netconn);
}

