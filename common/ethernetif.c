/*
 * ethernetif.c
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
#include <stddef.h>
#include <string.h>

#include <lwip/def.h>
#include <lwip/mem.h>
#include <lwip/pbuf.h>
#include <lwip/sys.h>
#include <lwip/stats.h>
#include <lwip/snmp.h>

#include <netif/etharp.h>
#include <netif/ppp_oe.h>

#include <ethernetif.h>

#include "sam4_uart.h"
#include "sam4_gmac.h"
#include "workqueue.h"

#define IFNAME0 'e'
#define IFNAME1 '0'

extern uint8_t mac_addr[6];

static struct pbuf *low_level_input(struct netif *nif)
{
	struct pbuf *p;

    p = gmac_rx(nif);
    if (!p)
    {
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);

        return NULL;
    }

    LINK_STATS_INC(link.recv);

    return p;
}

static void ethernetif_input(void *arg)
{
    struct netif *netif = arg;
    struct eth_hdr *ethhdr;
    struct pbuf *p;

    while ((p = low_level_input(netif)))
    {
        ethhdr = p->payload;

        switch (htons(ethhdr->type))
        {
        case ETHTYPE_IP:
        case ETHTYPE_ARP:
            if (netif->input(p, netif) != ERR_OK)
            {
                pbuf_free(p);
                p = NULL;
            }
            break;

        default:
            pbuf_free(p);
            p = NULL;
            break;
        }
    }
}

workqueue_t eth_rx_wq;
err_t ethernetif_init(struct netif *netif)
{
    netif->hwaddr_len = 6;
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    netif->mtu = 1500;

    eth_rx_wq.callback = ethernetif_input;
    eth_rx_wq.arg = netif;

    netif->output = etharp_output;
    netif->linkoutput = gmac_tx;

    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* set MAC hardware address */
    netif->hwaddr[0] = mac_addr[0];
    netif->hwaddr[1] = mac_addr[1];
    netif->hwaddr[2] = mac_addr[2];
    netif->hwaddr[3] = mac_addr[3];
    netif->hwaddr[4] = mac_addr[4];
    netif->hwaddr[5] = mac_addr[5];

    /* maximum transfer unit */
    netif->mtu = 1500;

    /* device capabilities */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    etharp_init();

    return ERR_OK;
}


