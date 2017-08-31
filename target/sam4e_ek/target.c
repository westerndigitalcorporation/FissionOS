/*
 * target.c
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
#include <lwip/dhcp.h>

#include "swd_target.h"
#include "swd.h"

#include "http.h"

#include "target.h"

#define HTTP_RECV_TIMEOUT_mS                     10000
#define PDI_ERROR                                -1
#define PDI_COMPLETE                             1

extern swd_t *swd_instance;

uint8_t pcie_power_on = 0;

// TODO:  Remove these when the pcie stuff is implemented
#define pcie_card_power()                        pcie_power_on
#define pcie_card_present()                      1

int http_target_xml(struct netconn *client, struct netbuf *conn_buf, char **querystr, void *arg)
{
    int ret;

    http_content_type_send(client, "application/xml");

    http_printf(client, "<targetstat>");
    http_printf(client, "<pciepower>");
    if (pcie_card_power())
    {
        http_printf(client, "true");
    }
    else
    {
        http_printf(client, "false");
    }
    http_printf(client, "</pciepower>");

    ret = swd_target_is_halted(swd_instance);
    if (ret >= 0)
    {
        if (ret)
        {
            http_printf(client, "true");
        }
        else
        {
            http_printf(client, "false");
        }
    }
    http_printf(client, "</targetstat>");
 
    return 0;
}

int cmd_target(struct netconn *client, struct netbuf *conn_buf, char **querystr, void *arg)
{
    char *haltedstr = "N/A";

    if (pcie_card_power())
    {
        int ret = swd_target_is_halted(swd_instance);

        if (ret >= 0)
        {
            if (!ret)
            {
                haltedstr = "False";
            }
            else
            {
                haltedstr = "True";
            }
        }
    }

    http_printf(client, "Target Configuration\r\n\n");
    http_printf(client, "  PCIe Card Present:  %s\r\n", pcie_card_present() ? "True" : "False");
    http_printf(client, "  PCIe Card Powered:  %s\r\n", pcie_card_power() ? "True" : "False");
    http_printf(client, "  CPU Halted       :  %s\r\n", haltedstr);
    http_printf(client, "\r\n");

    return 0;
}

int http_target_firmware(struct netconn *client, struct netbuf *rx_buf, char **querystr, void *arg)
{
    int total_len = 0;
    err_t err = ERR_OK;

    http_content_type_send(client, "text/plain");
    netconn_set_recvtimeout(client, HTTP_RECV_TIMEOUT_mS);

    while (err == ERR_OK)
    {
        uint8_t *data;
        uint16_t len;

        netbuf_first(rx_buf);
        netbuf_data(rx_buf, (void **)&data, &len);
        while (len)
        {
            total_len += len;

            // TODO:  Do something useful here.

            if (netbuf_next(rx_buf) != -1)   // Set buffer pointer forward
            {
                netbuf_data(rx_buf, (void **)&data, &len);
            }
            else
            {
                len = 0;
            }
        }

        netbuf_delete(rx_buf);   // Frees the netbuf list
        err = netconn_recv(client, &rx_buf);
    }

    return 0;
}

int http_target(struct netconn *client, struct netbuf *conn_buf, char **querystr, void *arg)
{
    char *key, *value;

    while (!http_request_key_value_next(querystr, &key, &value))
    {
        if (!strcmp("pciepoweron", key))
        {
            pcie_power_on = !strcmp("true", value) ? 1 : 0;
        }
    }

    http_content_type_send(client, "text/plain");

    http_printf(client, "<pre>");
    cmd_target(client, conn_buf, querystr, arg);
    http_printf(client, "</pre>");

    netbuf_delete(conn_buf);

    return 0;
}

