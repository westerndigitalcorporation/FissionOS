/*
 * saml_usb.c
 *
 *
 * Copyright (c) 2017 Jeremy Garff
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

#include "vectors.h"
#include "usb.h"
#include "saml_nvm.h"
#include "console.h"

#include "saml_usb.h"


/** Driver endpoint descriptor pointers.  */
static usb_endpoint_entry_t *usb_endpoint[USB_EP_NUM] = {};
static volatile usb_ep_desc_t usb_desc[USB_EP_NUM];
static uint8_t usb_addr = 0;

int usb_tx_overrun = 0;


/*
 * Debug console commands
 */
void show_ep_regs(console_t *console, volatile usb_ep_t *ep_regs)
{
    console_print(console, "  epcfg       : 0x%02x", ep_regs->epcfg);
    console_print(console, "  epstatusclr : 0x%02x", ep_regs->epstatusclr);
    console_print(console, "  epstatusset : 0x%02x", ep_regs->epstatusset);
    console_print(console, "  epstatus    : 0x%02x\r\n", ep_regs->epstatus);
    console_print(console, "  epintflag   : 0x%02x", ep_regs->epintflag);
    console_print(console, "  epintenclr  : 0x%02x", ep_regs->epintenclr);
    console_print(console, "  epintenset  : 0x%02x\r\n", ep_regs->epintenset);
}

void show_desc(console_t *console, volatile usb_ep_desc_t *desc)
{
    int j;

    for (j = 0; j < 2; j++)  // Two banks
    {
        volatile usb_ep_desc_bank_t *bank = &desc->bank[j];

        console_print(console, "  Desc %d\r\n", j);
        console_print(console, "    addr      %08x", bank->addr);
        console_print(console, "    pcksize   %08x\r\n", bank->pcksize);
        console_print(console, "    extreg    %08x", bank->extreg);
        console_print(console, "    status_bk %08x\r\n", bank->status_bk);
    }
}

int cmd_usb_regs(console_t *console, int argc, char *argv[])
{
    int i;

    console_print(console, "Stats:\r\n");
    console_print(console, "  usb_tx_overrun  : %d\r\n", usb_tx_overrun);
    console_print(console, "\n");
    console_print(console, "Main Registers:\r\n");
    console_print(console, "  ctrla           : 0x%02x\r\n", USB->ctrla);
    console_print(console, "  qosctrl         : 0x%02x\r\n", USB->qosctrl);
    console_print(console, "  ctrlb           : 0x%04x\r\n", USB->ctrlb);
    console_print(console, "  dadd            : 0x%02x\r\n", USB->dadd);
    console_print(console, "  status          : 0x%02x\r\n", USB->status);
    console_print(console, "  fsmstatus       : 0x%02x\r\n", USB->fsmstatus);
    console_print(console, "  fnum            : 0x%04x\r\n", USB->fnum);
    console_print(console, "  intflag         : 0x%04x\r\n", USB->intflag);
    console_print(console, "  intenset        : 0x%04x\r\n", USB->intenset);
    console_print(console, "  intenclr        : 0x%04x\r\n", USB->intenclr);
    console_print(console, "  epintsmry       : 0x%04x\r\n", USB->epintsmry);
    console_print(console, "  descadd         : 0x%08x\r\n", USB->descadd);
    console_print(console, "  padcal          : 0x%04x\r\n\n", USB->padcal);

    for (i = 0; i < USB_EP_NUM; i++)
    {
        volatile usb_ep_t *ep_regs = &USB->ep[i];

        if (ep_regs->epcfg) {
            console_print(console, "EP %d\r\n", i);
            show_ep_regs(console, ep_regs);
            show_desc(console, &usb_desc[i]);
        }

    }

    return 0;
}

int cmd_usb(console_t *console, int argc, char *argv[])
{
    if (argc < 2)
    {
        cmd_help_usage(console, argv[0]);
        return 0;
    }

    if ((argc == 2) && (!strcmp(argv[1], "show")))
    {
        return cmd_usb_regs(console, argc, argv);
    }
    else
    {
        cmd_help_usage(console, argv[0]);
    }

    return 0;
}

/**
 * Setup the hardware endpoint descriptor based on the provided
 * endpoint driver descriptor.
 *
 * @param    ep      Driver endpoint descriptor pointer.
 *
 * @returns  None
 */
void usb_endpoint_register(usb_endpoint_entry_t *ep)
{
    volatile usb_ep_t *ep_regs = &USB->ep[ep->num];
    volatile usb_ep_desc_bank_t *desc_in = &usb_desc[ep->num].bank[USB_EP_DESC_BANK_IN];
    volatile usb_ep_desc_bank_t *desc_out = &usb_desc[ep->num].bank[USB_EP_DESC_BANK_OUT];
    uint32_t irqstate = irq_save();

    usb_endpoint[ep->num] = ep;

    ep->read_bytes = 0;
    switch (ep->size)
    {
        case 16:
            desc_in->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_16);
            desc_out->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_16);
            break;

        case 32:
            desc_in->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_32);
            desc_out->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_32);
            break;

        case 64:
            desc_in->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_64);
            desc_out->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_64);
            break;

        case 128:
            desc_in->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_128);
            desc_out->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_128);
            break;

        default:
            desc_in->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_8);
            desc_out->pcksize = USB_EP_DESC_SIZE(USB_EP_DESC_SIZE_8);
            break;
    }

    if (ep->recv_buf)
    {
        desc_out->addr = (uint32_t)ep->recv_buf;
        desc_out->pcksize |= USB_EP_DESC_MULTI_PACKET_SIZE(ep->size);
    }

    switch (ep->type)
    {
        case USB_ENDPOINT_TYPE_CONTROL:
            ep_regs->epcfg = USB_EP_EPCFG_EPTYPE0(USB_EP_EPCFG_EPTYPE0_CTRL_OUT) |
                             USB_EP_EPCFG_EPTYPE1(USB_EP_EPCFG_EPTYPE1_CTRL_IN);
            break;

        case USB_ENDPOINT_TYPE_INT:
            ep_regs->epcfg = USB_EP_EPCFG_EPTYPE0(USB_EP_EPCFG_EPTYPE0_INT_OUT) |
                             USB_EP_EPCFG_EPTYPE1(USB_EP_EPCFG_EPTYPE1_INT_IN);
            break;

        case USB_ENDPOINT_TYPE_BULK:
            ep_regs->epcfg = USB_EP_EPCFG_EPTYPE0(USB_EP_EPCFG_EPTYPE0_BULK_OUT) |
                             USB_EP_EPCFG_EPTYPE1(USB_EP_EPCFG_EPTYPE1_BULK_IN);
            break;

        case USB_ENDPOINT_TYPE_ISO:
            ep_regs->epcfg = USB_EP_EPCFG_EPTYPE0(USB_EP_EPCFG_EPTYPE0_ISO_OUT) |
                             USB_EP_EPCFG_EPTYPE1(USB_EP_EPCFG_EPTYPE1_ISO_IN);
            break;
    }

    irq_restore(irqstate);
}

/**
 * Save the USB address provided.  This will be enabled at a later time
 * after the current transaction is complete.
 *
 * @param    addr    USB Address allocated from host.
 *
 * @returns  None
 */
void usb_address_set(uint8_t addr)
{
    usb_addr = addr;
}

/**
 * Activate the USB address.
 *
 * @returns  None
 */
void usb_address_enable(void)
{
    USB->dadd = USB_DEVICE_DADD_DADD(usb_addr) | USB_DEVICE_DADD_ADDEN;
}

/**
 * Attach the USB device to the host.  This will enable the pullups for
 * the configured speed.
 *
 * @returns  None
 */
void usb_attach(void)
{
    USB->ctrlb &= ~USB_DEVICE_CTRLB_DETACH;
}

/**
 * Attach the USB device to the host.  This will enable the pullups for
 * the configured speed.
 *
 * @returns  None
 */
void usb_detach(void)
{
    USB->ctrlb |= USB_DEVICE_CTRLB_DETACH;
}

/**
 *  Enable the RX/TX interrupt
 */
void usb_endpoint_rxstp_enable(usb_endpoint_entry_t *ep)
{
    volatile usb_ep_t *ep_regs = &USB->ep[ep->num];

    ep_regs->epintenset = USB_EP_EPINTENSET_RXSTP;
}

/**
 *  Enable the Setup RX interrupt
 */
void usb_endpoint_rxout_enable(usb_endpoint_entry_t *ep)
{
    volatile usb_ep_t *ep_regs = &USB->ep[ep->num];

    ep_regs->epintenset = USB_EP_EPINTENSET_TRCPT0;
}

/**
 * Given a endpoint, read any data from the hardware into the provided buffer.
 *
 * @param    ep        Driver endpoint descriptor pointer.
 * @param    buffer    Buffer pointer to copy data into.
 * @param    maxlen    Maximum number of bytes to copy.
 *
 * @returns  Number of bytes copied.
 */
int usb_endpoint_buffer_read(usb_endpoint_entry_t *ep, char *buffer, int maxlen)
{
    volatile usb_ep_desc_bank_t *desc = &usb_desc[ep->num].bank[USB_EP_DESC_BANK_OUT];
    volatile usb_ep_t *ep_regs = &USB->ep[ep->num];
    uint16_t recv_len = USB_EP_DESC_BYTE_COUNT_GET(desc->pcksize);
    uint8_t *src = (uint8_t *)desc->addr;
    int len = 0;
    int irqstate = irq_save();

    if (!buffer)
    {
        goto done;
    }

    if (!(ep_regs->epstatus & USB_EP_EPSTATUS_BK0RDY))
    {
        irq_restore(irqstate);
        return 0;
    }

    buffer = &buffer[ep->read_bytes];
    // TODO:  Speed the following up
    while ((len < maxlen) && (ep->read_bytes < recv_len))
    {
        *buffer++ = *src++;
        ep->read_bytes++;
        len++;
    }

    // Not done reading the buffer yet?
    if (ep->read_bytes < recv_len)
    {
        irq_restore(irqstate);
        return len;
    }

done:
    ep->read_bytes = 0;

    // Clear out the recv length
    desc->pcksize &= ~USB_EP_DESC_BYTE_COUNT(USB_EP_DESC_MULTI_PACKET_MAX);

    barrier();

    // Indicate that bank 0 is now ready for next receive
    ep_regs->epstatusclr = USB_EP_EPSTATUSCLR_BK0RDY;

    irq_restore(irqstate);

    return len;
}

/**
 * Given a endpoint, setup hardware to copy from this buffer on the next request..
 * Note:  This buffer is not copied, so its contents should not be changed until
 *        the transfer is complete.
 *
 * @param    ep        Driver endpoint descriptor pointer.
 * @param    buffer    Buffer pointer to data.
 * @param    len       Number of bytes to transfer.
 *
 * @returns  None
 */
void usb_endpoint_buffer_write(usb_endpoint_entry_t *ep, char *buffer, int len)
{
    volatile usb_ep_desc_bank_t *desc = &usb_desc[ep->num].bank[USB_EP_DESC_BANK_IN];
    volatile usb_ep_t *ep_regs = &USB->ep[ep->num];

    desc->addr = (uint32_t)buffer;

    desc->pcksize &= ~(USB_EP_DESC_MULTI_PACKET_SIZE(USB_EP_DESC_MULTI_PACKET_MAX) |
                       USB_EP_DESC_BYTE_COUNT(USB_EP_DESC_MULTI_PACKET_MAX));
    desc->pcksize |= USB_EP_DESC_BYTE_COUNT(len) | USB_EP_DESC_AUTO_ZLP;

    barrier();

	// Turn on interrupt and Indicate to hardware that the bank is now ready
    ep_regs->epintenset = USB_EP_EPINTENSET_TRCPT1;
    ep_regs->epstatusset = USB_EP_EPSTATUSSET_BK1RDY;
}

/**
 * Prepare hardware for the next IN transaction given a buffer.
 *
 * @param    ep        Driver endpoint descriptor pointer.
 * @param    buffer    Buffer pointer to data.
 * @param    len       Number of bytes to transfer.
 *
 * @returns  None
 */
int usb_txbuffer_start(usb_endpoint_entry_t *ep, char *buffer, int len)
{
    volatile usb_ep_t *ep_regs = &USB->ep[ep->num];
    int irqstate = irq_save();

    // Don't transfer if already in the middle
    if (ep_regs->epstatus & USB_EP_EPSTATUSSET_BK1RDY)
    {
        usb_tx_overrun++;
        irq_restore(irqstate);

        return -1;
    }

    ep->buffer = buffer;
    if (len > USB_EP_DESC_MULTI_PACKET_MAX)
    {
        len = USB_EP_DESC_MULTI_PACKET_MAX;
    }

    usb_endpoint_buffer_write(ep, buffer, len);

    irq_restore(irqstate);

    return len;
}

/**
 * Stub to remain compatible with mega32u driver.
 */
int usb_txbuffer_done(usb_endpoint_entry_t *ep)
{
    ep->buffer = NULL;

    return 0;
}

/**
 * Handle USB reset requests
 *
 * @returns  None
 */
void usb_reset(void)
{
    int i;

    usb_addr = 0;

    for (i = 0; i < USB_EP_NUM; i++)
    {
        if (usb_endpoint[i])
        {
            // Re-initialize the endpoint.  Hardware clears all endpoint configs _except_
            // control types during USB resets
            if (usb_endpoint[i]->type != USB_ENDPOINT_TYPE_CONTROL)
            {
                usb_endpoint_register(usb_endpoint[i]);
            }

            // Do reset callback if present
            if (usb_endpoint[i]->reset)
            {
                usb_endpoint[i]->reset(usb_endpoint[i]);
            }
        }
    }
}

uint32_t wrongnum = 0;
uint32_t wrongflags = 0;

/**
 * Interrupt handler for bus events.
 *
 * @returns  None
 */
void usb_int_handler(void)
{
    int i = 0;

    if (USB->intflag & USB_DEVICE_INTFLAG_EORST)
    {
        usb_reset();
        USB->intflag = USB_DEVICE_INTFLAG_EORST;
        barrier();

        return;
    }

    for (i = 0; i < ARRAY_SIZE(usb_endpoint); i++)
    {
        if (USB->epintsmry & (0x1 << i))
        {
            usb_endpoint_entry_t *ep = usb_endpoint[i];
            volatile usb_ep_t *ep_regs = &USB->ep[i];

            if (ep_regs->epintflag & USB_EP_EPINTFLAG_TRCPT0)           // OUT
            {
                if (ep->rx_out)
                {
                    // Clear interrupts prior to bank state clear to avoid
                    // interrupt loss
                    ep_regs->epintflag = USB_EP_EPINTFLAG_TRCPT0;

                    ep->rx_out(ep);
                }
            }

            if (ep_regs->epintflag & USB_EP_EPINTFLAG_RXSTP)            // Setup
            {
                if (ep->rx_setup)
                {
                    ep->rx_setup(ep);

                    // Do not clear the interrupt until after we've read the
                    // setup data.  The int flag is used for hardware to NAK
                    // setup requests.
                    ep_regs->epintflag = USB_EP_EPINTFLAG_RXSTP;
                }
            }

            if (ep_regs->epintflag & USB_EP_EPINTFLAG_TRCPT1)           // IN
            {
                if (ep->tx_in)
                {
                    // Clear interrupts prior to bank state clear to avoid
                    // interrupt loss
                    ep_regs->epintflag = USB_EP_EPINTFLAG_TRCPT1;

                    ep->tx_in(ep);
                }
            }
        }
    }
}

void usb_init(void)
{
    USB->descadd = (uint32_t)usb_desc;

    // Set the PAD calibration mode
    USB->padcal = USB_DEVICE_PADCAL_TRANSP(NVM_SOFT_CALIB_USB_TRANSP) |
                  USB_DEVICE_PADCAL_TRANSN(NVM_SOFT_CALIB_USB_TRANSN) |
                  USB_DEVICE_PADCAL_TRIM(NVM_SOFT_CALIB_USB_TRIM);

    // Enable the device - Full speed
    USB->ctrla = USB_DEVICE_CTRLA_ENABLE | USB_DEVICE_CTRLA_RUNSTBY;
    while (USB->syncbusy)
        ;

    // Build the endpoint descriptor list

    // Setup interrupt handler
    nvic_callback_set(PERIPHERAL_ID_USB, usb_int_handler);
    nvic_enable(PERIPHERAL_ID_USB);

#ifdef __ATSAMD53__
    nvic_callback_set(PERIPHERAL_ID_USB_SOF, usb_int_handler);
    nvic_enable(PERIPHERAL_ID_USB_SOF);

    nvic_callback_set(PERIPHERAL_ID_USB_TRCPT0, usb_int_handler);
    nvic_enable(PERIPHERAL_ID_USB_TRCPT0);

    nvic_callback_set(PERIPHERAL_ID_USB_TRCPT1, usb_int_handler);
    nvic_enable(PERIPHERAL_ID_USB_TRCPT1);
#endif /* __ATSAMD53__ */

    USB->intenset = USB_DEVICE_INTENSET_EORST;
}

/**
 * Disable the USB controller
 *
 * @returns  None
 */
void usb_fini(void)
{
    USB->ctrlb |= USB_DEVICE_CTRLB_DETACH;

    // Disable the device
    USB->ctrla = 0;
    while (USB->syncbusy)
        ;
}

