/*
 * sam4_gmac.c
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

#include "vectors.h"
#include "context.h"
#include "workqueue.h"
#include "console.h"

#include "sam4_clock.h"
#include "sam4_uart.h"
#include "sam4_gpio.h"

#include "lwip/netif.h"
#include "netif/etharp.h"

#include "sam4_gmac.h"


#define PHY_BMCR                      0x00
#define PHY_BMCR_FULL_DUPLEX          (1 << 8)   ///<
#define PHY_BMCR_AUTONEG_RESTART      (1 << 9)   ///<
#define PHY_BMCR_ISOLATE              (1 << 10)  ///< Isolate MII
#define PHY_BMCR_POWERDOWN            (1 << 11)
#define PHY_BMCR_AUTONEG_ENABLE       (1 << 12)
#define PHY_BMCR_SPEED_100            (1 << 13)
#define PHY_BMCR_LOOPBACK             (1 << 14)  ///<
#define PHY_BMCR_RESET                (1 << 15)  ///<

#define PHY_BMSR                      0x01
#define PHY_BMSR_LINKSTAT             (1 << 2)   ///<  Link status.
#define PHY_BMSR_ANEGCAPABLE          (1 << 3)   ///<  Able to do auto-negotiation
#define PHY_BMSR_ANCOMPL              (1 << 5)   ///<  Auto negotiation complete.

#define PHY_ANAR                      0x04
#define PHY_ANLPAR                    0x05       ///<  Auto negotiation link partner availability register.
#define PHY_ANLPAR_10_HDX             (1 << 5)   ///<  10BASE-T half duplex
#define PHY_ANLPAR_10_FDX             (1 << 6)   ///<  10BASE-T full duplex
#define PHY_ANLPAR_100TX_HDX          (1 << 7)   ///<  100BASE-TX half duplex
#define PHY_ANLPAR_100TX_FDX          (1 << 8)   ///<  100BASE-TX full duplex
#define PHY_ANLPAR_IEEE_802_3         (0x01 & 0x1f) ///< 802.3



static gmac_drv_t *gmac_instance;
static uint8_t gmac_phy_addr;

static thread_t *tx_thread;
extern workqueue_t eth_rx_wq;

static uint32_t gmac_intcount, gmac_rxcount, gmac_txcount;

/*
 * Interrupt handler
 */
static void gmac_irq(void)
{
    uint32_t status = GMAC->isr;
    uint32_t state = irq_save();

    gmac_intcount++;

    if ((status & (GMAC_IMR_TCOMP | GMAC_IMR_TFC |
                   GMAC_IMR_TUR)))
    {
        volatile gmac_txbuf_desc_t *desc = &gmac_instance->txdesc[gmac_instance->txbuf_hw];

        desc->status &= (GMAC_TXBUF_STATUS_WRAP | GMAC_TXBUF_STATUS_USED);

        gmac_txcount++;

        gmac_instance->txbuf_hw++;
        if (gmac_instance->txbuf_hw >= gmac_instance->txbuf_count)
        {
            gmac_instance->txbuf_hw = 0;
        }

        if (tx_thread)
        {
            thread_schedule(tx_thread);
            tx_thread = NULL;
        }
    }

    if ((status & (GMAC_IMR_RCOMP | GMAC_IMR_RXUBR |
                   GMAC_IMR_RLEX | GMAC_IMR_ROVR)))
    {
        workqueue_add(&eth_rx_wq, 0);
    }

    irq_restore(state);
}

/*
 * Phy Handling Routines
 */
static void gmac_man_enable(void)
{
	GMAC->ncr |= GMAC_NCR_MPE;
	GMAC->ncfgr |= GMAC_NCFGR_CLK_MCK_48;
}

static void gmac_man_disable(void)
{
	GMAC->ncr &= ~GMAC_NCR_MPE;
}

static uint16_t gmac_phy_read(uint8_t phy_addr, uint8_t reg)
{
    GMAC->man = GMAC_MAN_PHYA(phy_addr) |
                GMAC_MAN_REGA(reg) |
                GMAC_MAN_CLTTO |
                GMAC_MAN_READ |
                GMAC_MAN_WTN;

    while (!(GMAC->nsr & GMAC_NSR_IDLE))
        ;

    return (uint16_t)((GMAC->man & GMAC_MAN_DATA_MASK) >>
                       GMAC_MAN_DATA_SHIFT);
}

static void gmac_phy_write(uint8_t phy_addr, uint8_t reg, uint16_t data)
{
    GMAC->man = GMAC_MAN_PHYA(phy_addr) |
                GMAC_MAN_REGA(reg) |
                GMAC_MAN_DATA(data) |
                GMAC_MAN_CLTTO |
                GMAC_MAN_WRITE |
                GMAC_MAN_WTN;

    while (!(GMAC->nsr & GMAC_NSR_IDLE))
        ;
}

// TODO: Make a phy specific module that can be included from the GMAC
//       to be more board portable
int gmac_link_change_handle(void)
{
    uint32_t reg, ncfgr = 0;

    gmac_man_enable();

    // Override stapping for auto-negotiation 100/10 FD/HD supported.
    reg = gmac_phy_read(gmac_phy_addr, PHY_BMCR);
    reg &= ~(PHY_BMCR_AUTONEG_ENABLE | PHY_BMCR_LOOPBACK | PHY_BMCR_POWERDOWN);
    reg |= PHY_BMCR_ISOLATE;
    gmac_phy_write(gmac_phy_addr, PHY_BMCR, reg);

    reg = PHY_ANLPAR_100TX_HDX | PHY_ANLPAR_100TX_FDX |
          PHY_ANLPAR_10_HDX | PHY_ANLPAR_10_FDX |
          PHY_ANLPAR_IEEE_802_3;
    gmac_phy_write(gmac_phy_addr, PHY_ANAR, reg);

    reg = gmac_phy_read(gmac_phy_addr, PHY_BMCR);
    reg |= PHY_BMCR_SPEED_100 | PHY_BMCR_FULL_DUPLEX | PHY_BMCR_AUTONEG_ENABLE;
    gmac_phy_write(gmac_phy_addr, PHY_BMCR, reg);

    // Restart auto-negotiation
    reg &= ~PHY_BMCR_ISOLATE;
    reg |= PHY_BMCR_AUTONEG_RESTART;
    gmac_phy_write(gmac_phy_addr, PHY_BMCR, reg);

    // Wait for autonegotiation complete
    do
    {
        reg = gmac_phy_read(gmac_phy_addr, PHY_BMSR);
    } while (!(reg & PHY_BMSR_ANCOMPL));

    // Read link status from partner
    reg = gmac_phy_read(gmac_phy_addr, PHY_ANLPAR);

    // Configure speed
	if ((reg & PHY_ANLPAR_100TX_FDX) || (reg & PHY_ANLPAR_100TX_HDX))
	{
        ncfgr |= GMAC_NCFGR_SPD;
	}
	else
	{
        ncfgr &= ~GMAC_NCFGR_SPD;
	}

    // Configure duplex
	if ((reg & PHY_ANLPAR_100TX_FDX) || (reg & PHY_ANLPAR_10_FDX))
	{
        ncfgr |= GMAC_NCFGR_FD;
	}
	else
	{
        ncfgr &= ~GMAC_NCFGR_FD;
	}

    GMAC->ncfgr = ncfgr;

    gmac_man_disable();

    // For loopback mode debugging
    //GMAC->ncr |= GMAC_NCR_LB | GMAC_NCR_LBL;

    // Start transmit/recieve
    GMAC->ncr |= GMAC_NCR_RXEN | GMAC_NCR_TXEN | GMAC_NCR_WESTAT;

    return 0;
}

/*
 * Receive and transmit
 */
static volatile gmac_txbuf_desc_t *gmac_txdesc_next(gmac_drv_t *gmac)
{
    gmac->txbuf_next++;
    if (gmac->txbuf_next >= gmac->txbuf_count)
    {
        gmac->txbuf_next = 0;
    }

    return &gmac->txdesc[gmac->txbuf_next];
}

err_t gmac_tx(struct netif *netif, struct pbuf *p)
{
    gmac_drv_t *gmac = netif_to_gmac(netif);
    uint32_t state = irq_save();
    volatile gmac_txbuf_desc_t *desc;
    int offset = 0;
    struct pbuf *q;

    desc = &gmac->txdesc[gmac->txbuf_next];
    while (desc->status & GMAC_TXBUF_STATUS_LAST)
    {
        tx_thread = thread_current;
        irq_restore(state);
        thread_pend_current();
        state = irq_save();

        desc = &gmac->txdesc[gmac->txbuf_next];
    }

	for (q = p; q != NULL; q = q->next)
    {
        uint8_t *txbuf = (uint8_t *)desc->addr;

        memcpy(&txbuf[offset], q->payload, q->len);
        offset += q->len;
    }

    desc->status &= GMAC_TXBUF_STATUS_WRAP;
    desc->status |= GMAC_TXBUF_STATUS_LAST |
                    GMAC_TXBUF_STATUS_LEN(offset);

    gmac_txdesc_next(gmac);

    GMAC->ncr |= GMAC_NCR_TSTART;

    irq_restore(state);

    return ERR_OK;
}

static volatile gmac_rxbuf_desc_t *gmac_rxdesc_next(gmac_drv_t *gmac)
{
    gmac->rxbuf_next++;
    if (gmac->rxbuf_next >= gmac->rxbuf_count)
    {
        gmac->rxbuf_next = 0;
    }

    return &gmac->rxdesc[gmac->rxbuf_next];
}

struct pbuf *gmac_rx(struct netif *netif)
{
    gmac_drv_t *gmac = netif_to_gmac(netif);
    uint32_t state = irq_save();
    volatile gmac_rxbuf_desc_t *desc = &gmac->rxdesc[gmac->rxbuf_next];
    struct pbuf *p = NULL;
    int len;

    if (!(desc->addr & GMAC_RXBUF_ADDR_USED))
    {
        irq_restore(state);
        return NULL;
    }

    len = desc->status & GMAC_RXBUF_STATUS_LEN;
    if (len)
    {
        struct pbuf *q;
        int offset = 0;

        p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
        if (p)
        {
            for (q = p; q != NULL; q = q->next)
            {
                uint8_t *bufptr = (uint8_t *)(GMAC_RXBUF_ADDR(desc->addr) + offset);

                memcpy(q->payload, bufptr, q->len);
                offset += q->len;
            }
        }

        gmac_rxcount++;
    }

    gmac_rxdesc_next(gmac);

    desc->status = 0;
    desc->addr &= ~GMAC_RXBUF_ADDR_USED;

    irq_restore(state);
    return p;
}


/*
 * Initialization Functions
 */
static void gmac_tx_init(gmac_drv_t *gmac, uint8_t *pool)
{
    int i;

    for (i = 0; i < gmac->txbuf_count; i++)
    {
        gmac->txdesc[i].addr = (uint32_t)pool;
        gmac->txdesc[i].status = GMAC_TXBUF_STATUS_USED;
        pool += GMAC_TXBUF_SIZE;
    }

    gmac->txdesc[gmac->txbuf_count - 1].status |= GMAC_TXBUF_STATUS_WRAP;
    gmac->txbuf_next = 0;
    gmac->tx_offset = 0;

    GMAC->tbqb = (uint32_t)gmac->txdesc;
    GMAC->tsr = GMAC_TSR_COL | GMAC_TSR_RLE | GMAC_TSR_UND |
                GMAC_TSR_TXCOMP;
}

static void gmac_rx_init(gmac_drv_t *gmac, uint8_t *pool)
{
    int i;

    for (i = 0; i < gmac->rxbuf_count; i++)
    {
        gmac->rxdesc[i].addr = GMAC_RXBUF_ADDR((uint32_t)pool);
        gmac->rxdesc[i].status = 0;
        pool += GMAC_RXBUF_SIZE;
    }

    gmac->rxdesc[gmac->rxbuf_count - 1].addr |= GMAC_RXBUF_ADDR_WRAP;
    gmac->rxbuf_next = 0;
    gmac->rx_offset = 0;

    GMAC->dcfgr &= ~GMAC_DCFGR_DRBS_MASK;
    GMAC->dcfgr |= GMAC_DCFGR_DRBS(GMAC_RXBUF_SIZE / 64);
    GMAC->rbqb = (uint32_t)gmac->rxdesc;
    GMAC->rsr = GMAC_RSR_RXOVR | GMAC_RSR_REC | GMAC_RSR_BNA;
}

void gmac_init(gmac_drv_t *gmac, uint8_t *mac_addr, uint8_t phy_addr,
               uint8_t *txpool, uint8_t *rxpool)
{
    clock_peripheral_start(PERIPHERAL_ID_EMAC);

    gmac_instance = gmac;
    gmac_phy_addr = phy_addr;

    // TODO:  The following should use defined port values
    GPIOD->pudr = 0x1ffff; // Turn off pull ups
    GPIOD->pdr = 0x1ffff;   // All 16 ethernet pins on PORTD

    // Setup the hardware MAC address for packet filtering
    memcpy((uint8_t *)GMAC->sab[0], mac_addr, ETHARP_HWADDR_LEN);

    // Required according to the confusing documentation
    GMAC->ur = GMAC_UR_RMIIMII;

    gmac_tx_init(gmac, txpool);
    gmac_rx_init(gmac, rxpool);

    // Configure interrupts
    GMAC->idr = ~0UL;
    nvic_callback_set(PERIPHERAL_ID_EMAC, gmac_irq);
    nvic_enable(PERIPHERAL_ID_EMAC);
    GMAC->ier = GMAC_IMR_TCOMP | GMAC_IMR_TUR | GMAC_IMR_TFC | GMAC_IMR_TXUBR |
                GMAC_IMR_PFTR | GMAC_IMR_RCOMP | GMAC_IMR_ROVR | GMAC_IMR_RLEX;
}

/*
 * Console/debug
 */
int cmd_eth_desc(console_t *console)
{
    int i;

    console_print(console, "TX Descriptors:  Next %d\r\n", gmac_instance->txbuf_next);
    for (i = 0; i < gmac_instance->txbuf_count; i++)
    {
        volatile gmac_txbuf_desc_t *desc = &gmac_instance->txdesc[i];

        console_print(console, "    %d: %08x %08x\r\n", i, desc->addr, desc->status);
    }

    console_print(console, "RX Descriptors:  Next %d\r\n", gmac_instance->rxbuf_next);
    for (i = 0; i < gmac_instance->rxbuf_count; i++)
    {
        volatile gmac_rxbuf_desc_t *desc = &gmac_instance->rxdesc[i];
        console_print(console, "    %d: %08x %08x\r\n", i, desc->addr, desc->status);
    }

    return 0;
}

int cmd_eth_regs(console_t *console)
{
    int i;

    for (i = 0; i < 0x40; i += sizeof(uint32_t))
    {
        console_print(console, "    %03x: %08x", i, *(volatile uint32_t *)(GMAC_ADDR + i));
        if (((i + 4) % (4 * 4)) == 0)
        {
            console_print(console, "\r\n");
        }
    }

    for (i = 0x80; i < 0xd0; i += sizeof(uint32_t))
    {
        console_print(console, "    %03x: %08x", i, *(volatile uint32_t *)(GMAC_ADDR + i));
        if (((i + 4) % (4 * 4)) == 0)
        {
            console_print(console, "\r\n");
        }
    }

    for (i = 0x100; i < 0x200; i += sizeof(uint32_t))
    {
        console_print(console, "    %03x: %08x", i, *(volatile uint32_t *)(GMAC_ADDR + i));
        if (((i + 4) % (4 * 4)) == 0)
        {
            console_print(console, "\r\n");
        }
    }

    return 0;
}

int cmd_eth_link(console_t *console)
{
    uint16_t reg;

    gmac_man_enable();

    reg = gmac_phy_read(gmac_phy_addr, PHY_BMSR);
    if (reg & PHY_BMSR_ANCOMPL)
    {
        reg = gmac_phy_read(gmac_phy_addr, PHY_BMSR);
        console_print(console, "Link      : %s%s\r\n",
                      reg & (PHY_ANLPAR_100TX_FDX || PHY_ANLPAR_100TX_HDX) ?
                      "100" : "10",
                      reg & (PHY_ANLPAR_100TX_FDX || PHY_ANLPAR_10_FDX) ?
                      "FD" : "HD");
    }
    else
    {
        console_print(console, "  No Link\r\n");
    }

    gmac_man_disable();

    return 0;
}

#define PHY_REG_COUNT                            32
int cmd_eth_phy(console_t *console)
{
    int i;

    gmac_man_enable();

    for (i = 0; i < PHY_REG_COUNT; i++)
    {
        console_print(console, "    %2x: %04x", i, gmac_phy_read(gmac_phy_addr, i));
        if (((i + 1) % 4) == 0)
        {
            console_print(console, "\r\n");
        }
    }
    gmac_man_disable();

    return 0;
}

int cmd_eth_ints(console_t *console)
{
    console_print(console, "Interrupts: %d\r\n", gmac_intcount);
    console_print(console, "RX Packets: %d\r\n", gmac_rxcount);
    console_print(console, "TX Packets: %d\r\n", gmac_txcount);

    return 0;
}

int cmd_eth(console_t *console, int argc, char *argv[])
{
    if (argc == 2)
    {
        if (!strcmp("phy", argv[1]))
        {
            cmd_eth_link(console);
            cmd_eth_phy(console);
        } else if (!strcmp("regs", argv[1]))
        {
            cmd_eth_regs(console);
        } else if (!strcmp("ints", argv[1]))
        {
            cmd_eth_ints(console);
        } else if (!strcmp("desc", argv[1]))
        {
            cmd_eth_desc(console);
        }
        return 0;
    }

    cmd_eth_link(console);
    cmd_eth_ints(console);

    return 0;
}

