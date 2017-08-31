/*
 * sam4_gmac.h
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


#ifndef __SAM4_GMAC_H__
#define __SAM4_GMAC_H__

#include <stddef.h>

#include "lwip/netif.h"

typedef struct gmac
{
    uint32_t ncr;
#define GMAC_NCR_LB                              (1 << 0)
#define GMAC_NCR_LBL                             (1 << 1)
#define GMAC_NCR_RXEN                            (1 << 2)
#define GMAC_NCR_TXEN                            (1 << 3)
#define GMAC_NCR_MPE                             (1 << 4)
#define GMAC_NCR_CRLSTAT                         (1 << 5)
#define GMAC_NCR_INCSTAT                         (1 << 6)
#define GMAC_NCR_WESTAT                          (1 << 7)
#define GMAC_NCR_BP                              (1 << 8)
#define GMAC_NCR_TSTART                          (1 << 9)
#define GMAC_NCR_THALT                           (1 << 10)
#define GMAC_NCR_TXPF                            (1 << 11)
#define GMAC_NCR_TXZQPF                          (1 << 12)
#define GMAC_NCR_RDS                             (1 << 14)
#define GMAC_NCR_SRTSM                           (1 << 15)
#define GMAC_ENPBPR                              (1 << 16)
#define GMAC_NCR_TXPBPF                          (1 << 17)
#define GMAC_NCR_FNP                             (1 << 18)
    uint32_t ncfgr;
#define GMAC_NCFGR_SPD                           (1 << 0)
#define GMAC_NCFGR_FD                            (1 << 1)
#define GMAC_NCFGR_DNVLAN                        (1 << 2)
#define GMAC_NCFGR_JFRAME                        (1 << 3)
#define GMAC_NCFGR_CAF                           (1 << 4)
#define GMAC_NCFGR_NBC                           (1 << 5)
#define GMAC_NCFGR_MTIHEN                        (1 << 6)
#define GMAC_NCFGR_UNIHEN                        (1 << 7)
#define GMAC_NCFGR_MAXFS                         (1 << 8)
#define GMAC_NCFGR_RTY                           (1 << 12)
#define GMAC_NCFGR_PEN                           (1 << 13)
#define GMAC_NCFGR_RXBUFO(val)                   ((val & 0x3) << 14)
#define GMAC_NCFGR_LFERD                         (1 << 16)
#define GMAC_NCFGR_RFCS                          (1 << 17)
#define GMAC_NCFGR_CLK_MCK_8                     (0 << 18)
#define GMAC_NCFGR_CLK_MCK_16                    (1 << 18)
#define GMAC_NCFGR_CLK_MCK_32                    (2 << 18)
#define GMAC_NCFGR_CLK_MCK_48                    (3 << 18)
#define GMAC_NCFGR_CLK_MCK_64                    (4 << 18)
#define GMAC_NCFGR_CLK_MCK_96                    (5 << 18)
#define GMAC_NCFGR_DBW(val)                      ((val & 0x3) << 0)
#define GMAC_NCFGR_DCPF                          (1 << 23)
#define GMAC_NCFGR_RXCOEN                        (1 << 24)
#define GMAC_NCFGR_EFRHD                         (1 << 25)
#define GMAC_NCFGR_IRXFCS                        (1 << 26)
#define GMAC_NCFGR_IPGSEN                        (1 << 28)
#define GMAC_NCFGR_RXBP                          (1 << 29)
#define GMAC_NCFGR_IRXER                         (1 << 30)
    uint32_t nsr;
#define GMAC_NSR_MDIO                            (1 << 1)
#define GMAC_NSR_IDLE                            (1 << 2)
    uint32_t ur;
#define GMAC_UR_RMIIMII                          (1 << 0)
#define GMAC_UR_HDFC                             (1 << 6)
#define GMAC_UR_BPDG                             (1 << 7)
    uint32_t dcfgr;
#define GMAC_DCFGR_FBLDO(val)                    ((val & 0x1f) << 0)
#define GMAC_DCFGR_ESMA                          (1 << 6)
#define GMAC_DCFGR_ESPA                          (1 << 7)
#define GMAC_DCFGR_TXCOEN                        (1 << 11)
#define GMAC_DCFGR_DRBS_MASK                     (0xff << 16)
#define GMAC_DCFGR_DRBS(val)                     ((val & 0xff) << 16)
    uint32_t tsr;
#define GMAC_TSR_UBR                             (1 << 0)
#define GMAC_TSR_COL                             (1 << 1)
#define GMAC_TSR_RLE                             (1 << 2)
#define GMAC_TSR_TXGO                            (1 << 3)
#define GMAC_TSR_TFC                             (1 << 4)
#define GMAC_TSR_TXCOMP                          (1 << 5)
#define GMAC_TSR_UND                             (1 << 6)
#define GMAC_TSR_HRESP                           (1 << 8)
    uint32_t rbqb;
    uint32_t tbqb;
    uint32_t rsr;
#define GMAC_RSR_BNA                             (1 << 0)
#define GMAC_RSR_REC                             (1 << 1)
#define GMAC_RSR_RXOVR                           (1 << 2)
#define GMAC_RSR_HNO                             (1 << 3)
    uint32_t isr;
    uint32_t ier;
    uint32_t idr;
    uint32_t imr;
#define GMAC_IMR_MFS                             (1 << 0)
#define GMAC_IMR_RCOMP                           (1 << 1)
#define GMAC_IMR_RXUBR                           (1 << 2)
#define GMAC_IMR_TXUBR                           (1 << 3)
#define GMAC_IMR_TUR                             (1 << 4)
#define GMAC_IMR_RLEX                            (1 << 5)
#define GMAC_IMR_TFC                             (1 << 6)
#define GMAC_IMR_TCOMP                           (1 << 7)
#define GMAC_IMR_ROVR                            (1 << 10)
#define GMAC_IMR_HRESP                           (1 << 11)
#define GMAC_IMR_PFNZ                            (1 << 12)
#define GMAC_IMR_PTZ                             (1 << 13)
#define GMAC_IMR_PFTR                            (1 << 14)
#define GMAC_IMR_EXINT                           (1 << 15)
#define GMAC_IMR_DRQFR                           (1 << 18)
#define GMAC_IMR_SFR                             (1 << 19)
#define GMAC_IMR_DRQFT                           (1 << 20)
#define GMAC_IMR_SFT                             (1 << 21)
#define GMAC_IMR_PDRQFR                          (1 << 22)
#define GMAC_IMR_PDRSFR                          (1 << 23)
#define GMAC_IMR_PDRQFT                          (1 << 24)
#define GMAC_IMR_PDRSFT                          (1 << 25)
    uint32_t man;
#define GMAC_MAN_DATA_SHIFT                      0
#define GMAC_MAN_DATA_MASK                       (0xffff)
#define GMAC_MAN_DATA(val)                       ((val & GMAC_MAN_DATA_MASK) << GMAC_MAN_DATA_SHIFT)
#define GMAC_MAN_WTN                             (2 << 16)
#define GMAC_MAN_REGA(val)                       ((val & 0x1f) << 18)
#define GMAC_MAN_PHYA(val)                       ((val & 0x1f) << 23)
#define GMAC_MAN_WRITE                           (1 << 28)
#define GMAC_MAN_READ                            (2 << 28)
#define GMAC_MAN_CLTTO                           (1 << 30)
#define GMAC_MAN_WZO                             (1 << 31)
    uint32_t rpq;
    uint32_t tpq;
    uint32_t resvd_0x40[16];
    uint32_t hrb;
    uint32_t hrt;
    uint8_t sab[4][8];
    uint32_t tidm[4];
    uint32_t ipgs;
    uint32_t svlan;
    uint32_t tpfcp;
    uint8_t samb[8];
} __attribute__ ((packed)) gmac_t;

#define GMAC_ADDR                                0x40034000
#define GMAC                                     ((volatile gmac_t *)0x40034000)

typedef struct gmac_rxbuf_desc
{
    uint32_t addr;
#define GMAC_RXBUF_ADDR_USED                     (1 << 0)
#define GMAC_RXBUF_ADDR_WRAP                     (1 << 1)
#define GMAC_RXBUF_ADDR(val)                     (val & 0xfffffffc)
    uint32_t status;
#define GMAC_RXBUF_STATUS_LEN                    (0x0fff)
#define GMAC_RXBUF_STATUS_SOF                    (1 << 14)
#define GMAC_RXBUF_STATUS_EOF                    (1 << 15)
#define GMAC_RXBUF_STATUS_CFI                    (1 << 16)
} __attribute__ ((packed, aligned(8))) gmac_rxbuf_desc_t;

#define GMAC_RXBUF_SIZE                          1536

typedef struct gmac_txbuf_desc
{
    uint32_t addr;
    uint32_t status;
#define GMAC_TXBUF_STATUS_LEN_MASK               0x1fff
#define GMAC_TXBUF_STATUS_LEN(val)               ((val & 0x1fff) << 0)
#define GMAC_TXBUF_STATUS_LAST                   (1 << 15)
#define GMAC_TXBUF_STATUS_WRAP                   (1 << 30)
#define GMAC_TXBUF_STATUS_USED                   (1 << 31)
} __attribute__ ((packed, aligned(8))) gmac_txbuf_desc_t;

#define GMAC_TXBUF_SIZE                          1536

typedef struct
{
    struct netif netif;
    volatile uint32_t flags;
#define GMAC_FLAGS_TX_BUSY                       0x01

    volatile gmac_txbuf_desc_t *txdesc;
    uint8_t txbuf_count;
    volatile uint8_t txbuf_hw;
    volatile uint8_t txbuf_next;
    uint16_t tx_offset;

    volatile gmac_rxbuf_desc_t *rxdesc;
    uint8_t rxbuf_count;
    volatile uint8_t rxbuf_next;
    uint16_t rx_offset;
} gmac_drv_t;

static inline gmac_drv_t *netif_to_gmac(struct netif *nif)
{
    return ((gmac_drv_t *)((uint32_t)nif - offsetof(gmac_drv_t, netif)));
}

int gmac_link_change_handle(void);
void gmac_init(gmac_drv_t *gmac, uint8_t *mac_addr, uint8_t phy_addr,
               uint8_t *txpool, uint8_t *rxpool);
int cmd_eth(uart_drv_t *uart, int argc, char *argv[]);

struct pbuf *gmac_rx(struct netif *netif);
err_t gmac_tx(struct netif *netif, struct pbuf *p);

#endif
