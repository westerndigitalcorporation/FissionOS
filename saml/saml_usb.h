/*
 * saml_usb.h
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


#ifndef __SAML_USB_H__
#define __SAML_USB_H__

#include "saml_sercom.h"

#include <console.h>


#define USB_EP_NUM                               8

typedef struct usb_ep
{
    uint8_t  epcfg;
#define USB_EP_EPCFG_EPTYPE0(val)                (((val) & 0x7) << 0)
#define USB_EP_EPCFG_EPTYPE0_DISABLED            0x0
#define USB_EP_EPCFG_EPTYPE0_CTRL_OUT            0x1
#define USB_EP_EPCFG_EPTYPE0_ISO_OUT             0x2
#define USB_EP_EPCFG_EPTYPE0_BULK_OUT            0x3
#define USB_EP_EPCFG_EPTYPE0_INT_OUT             0x4
#define USB_EP_EPCFG_EPTYPE0_DUAL_BANK_IN        0x5
#define USB_EP_EPCFG_EPTYPE1(val)                (((val) & 0x7) << 4)
#define USB_EP_EPCFG_EPTYPE1_DISABLED            0x0
#define USB_EP_EPCFG_EPTYPE1_CTRL_IN             0x1
#define USB_EP_EPCFG_EPTYPE1_ISO_IN              0x2
#define USB_EP_EPCFG_EPTYPE1_BULK_IN             0x3
#define USB_EP_EPCFG_EPTYPE1_INT_IN              0x4
#define USB_EP_EPCFG_EPTYPE1_DUAL_BANK_OUT       0x5
    uint8_t  reserved_0x01[3];
    uint8_t  epstatusclr;
#define USB_EP_EPSTATUSCLR_DTGLOUT               (1 << 0)
#define USB_EP_EPSTATUSCLR_DTGLIN                (1 << 1)
#define USB_EP_EPSTATUSCLR_CURBK                 (1 << 2)
#define USB_EP_EPSTATUSCLR_STALLRQ0              (1 << 4)
#define USB_EP_EPSTATUSCLR_STALLRQ1              (1 << 5)
#define USB_EP_EPSTATUSCLR_BK0RDY                (1 << 6)
#define USB_EP_EPSTATUSCLR_BK1RDY                (1 << 7)
    uint8_t  epstatusset;
#define USB_EP_EPSTATUSSET_DTGLOUT               (1 << 0)
#define USB_EP_EPSTATUSSET_DTGLIN                (1 << 1)
#define USB_EP_EPSTATUSSET_CURBK                 (1 << 2)
#define USB_EP_EPSTATUSSET_STALLRQ0              (1 << 4)
#define USB_EP_EPSTATUSSET_STALLRQ1              (1 << 5)
#define USB_EP_EPSTATUSSET_BK0RDY                (1 << 6)
#define USB_EP_EPSTATUSSET_BK1RDY                (1 << 7)
    uint8_t  epstatus;
#define USB_EP_EPSTATUS_DTGLOUT                  (1 << 0)
#define USB_EP_EPSTATUS_DTGLIN                   (1 << 1)
#define USB_EP_EPSTATUS_CURBK                    (1 << 2)
#define USB_EP_EPSTATUS_STALLRQ0                 (1 << 4)
#define USB_EP_EPSTATUS_STALLRQ1                 (1 << 5)
#define USB_EP_EPSTATUS_BK0RDY                   (1 << 6)
#define USB_EP_EPSTATUS_BK1RDY                   (1 << 7)
    uint8_t  epintflag;
#define USB_EP_EPINTFLAG_TRCPT0                  (1 << 0)
#define USB_EP_EPINTFLAG_TRCPT1                  (1 << 1)
#define USB_EP_EPINTFLAG_TRFAIL0                 (1 << 2)
#define USB_EP_EPINTFLAG_TRFAIL1                 (1 << 3)
#define USB_EP_EPINTFLAG_RXSTP                   (1 << 4)
#define USB_EP_EPINTFLAG_STALL                   (1 << 5)
    uint8_t  epintenclr;
#define USB_EP_EPINTENCLR_TRCPT0                 (1 << 0)
#define USB_EP_EPINTENCLR_TRCPT1                 (1 << 1)
#define USB_EP_EPINTENCLR_TRFAIL0                (1 << 2)
#define USB_EP_EPINTENCLR_TRFAIL1                (1 << 3)
#define USB_EP_EPINTENCLR_RXSTP                  (1 << 4)
#define USB_EP_EPINTENCLR_STALL                  (1 << 5)
    uint8_t  epintenset;
#define USB_EP_EPINTENSET_TRCPT0                 (1 << 0)
#define USB_EP_EPINTENSET_TRCPT1                 (1 << 1)
#define USB_EP_EPINTENSET_TRFAIL0                (1 << 2)
#define USB_EP_EPINTENSET_TRFAIL1                (1 << 3)
#define USB_EP_EPINTENSET_RXSTP                  (1 << 4)
#define USB_EP_EPINTENSET_STALL                  (1 << 5)
    uint8_t  reserved_0x0a[0x16];
} __attribute__ ((packed)) usb_ep_t;

typedef struct usb_device
{
    uint8_t  ctrla;
#define USB_DEVICE_CTRLA_SWRST                   (1 << 0)
#define USB_DEVICE_CTRLA_ENABLE                  (1 << 1)
#define USB_DEVICE_CTRLA_RUNSTBY                 (1 << 2)
#define USB_DEVICE_CTRLA_MODE                    (1 << 7)
    uint8_t  resvd_0x01;
    uint8_t  syncbusy;
#define USB_DEVICE_SYNCBUSY_SWRST                (1 << 0)
#define USB_DEVICE_SYNCBUSY_ENABLE               (1 << 1)
    uint8_t  qosctrl;
#define USB_DEVICE_OSCTRL_CQOS(val)              (((val) & 0x3) << 0)
#define USB_DEVICE_OSCTRL_DQOS(val)              (((val) & 0x3) << 2)
    uint32_t resvd_0x04;
    uint16_t ctrlb;
#define USB_DEVICE_CTRLB_DETACH                  (1 << 0)
#define USB_DEVICE_CTRLB_UPRSM                   (1 << 1)
#define USB_DEVICE_CTRLB_SPDCONF(val)            (((val) & 0x3) << 2)
#define USB_DEVICE_CTRLB_NREPLY                  (1 << 4)
#define USB_DEVICE_CTRLB_GNAK                    (1 << 9)
#define USB_DEVICE_CTRLB_LPMHDSK(val)            (((val) & 0x3) << 10)
    uint8_t  dadd;
#define USB_DEVICE_DADD_DADD(val)                (((val) & 0x7f) << 0)
#define USB_DEVICE_DADD_ADDEN                    (1 << 7)
    uint8_t  resvd_0x0b;
    uint8_t  status;
#define USB_DEVICE_STATUS_SPEED                  (((val) & 0x03) << 2)
#define USB_DEVICE_STATUS_LINESTATE              (((val) & 0x03) << 6)
    uint8_t  fsmstatus;
#define USB_DEVICE_FSMSTATUS_FSMSTATE(val)       (((val) & 0x7f) << 0)
    uint16_t resvd_0x0e;
    uint16_t fnum;
#define USB_DEVICE_FNUM_FNUM(val)                (((val) & 0x7ff) << 3)
#define USB_DEVICE_FNUM_FNCERR                   (1 << 15)
    uint16_t resvd_0x12;
    uint16_t intenclr;
#define USB_DEVICE_INTENCLR_SUSPEND              (1 << 0)
#define USB_DEVICE_INTENCLR_SOF                  (1 << 2)
#define USB_DEVICE_INTENCLR_EORST                (1 << 3)
#define USB_DEVICE_INTENCLR_WAKEUP               (1 << 4)
#define USB_DEVICE_INTENCLR_EORSM                (1 << 5)
#define USB_DEVICE_INTENCLR_UPRSM                (1 << 6)
#define USB_DEVICE_INTENCLR_RAMACER              (1 << 7)
#define USB_DEVICE_INTENCLR_LPMNYET              (1 << 8)
#define USB_DEVICE_INTENCLR_LPMSUSP              (1 << 9)
    uint16_t resvd_0x16;
    uint16_t intenset;
#define USB_DEVICE_INTENSET_SUSPEND              (1 << 0)
#define USB_DEVICE_INTENSET_SOF                  (1 << 2)
#define USB_DEVICE_INTENSET_EORST                (1 << 3)
#define USB_DEVICE_INTENSET_WAKEUP               (1 << 4)
#define USB_DEVICE_INTENSET_EORSM                (1 << 5)
#define USB_DEVICE_INTENSET_UPRSM                (1 << 6)
#define USB_DEVICE_INTENSET_RAMACER              (1 << 7)
#define USB_DEVICE_INTENSET_LPMNYET              (1 << 8)
#define USB_DEVICE_INTENSET_LPMSUSP              (1 << 9)
    uint16_t resvd_0x1a;
    uint16_t intflag;
#define USB_DEVICE_INTFLAG_SUSPEND               (1 << 0)
#define USB_DEVICE_INTFLAG_SOF                   (1 << 2)
#define USB_DEVICE_INTFLAG_EORST                 (1 << 3)
#define USB_DEVICE_INTFLAG_WAKEUP                (1 << 4)
#define USB_DEVICE_INTFLAG_EORSM                 (1 << 5)
#define USB_DEVICE_INTFLAG_UPRSM                 (1 << 6)
#define USB_DEVICE_INTFLAG_RAMACER               (1 << 7)
#define USB_DEVICE_INTFLAG_LPMNYET               (1 << 8)
#define USB_DEVICE_INTFLAG_LPMSUSP               (1 << 9)
    uint16_t resvd_0x1e;
    uint16_t epintsmry;
    uint16_t resvd_0x22;
    uint32_t descadd;
    uint16_t padcal;
#define USB_DEVICE_PADCAL_TRANSP(val)            (((val) & 0x1f) << 0)
#define USB_DEVICE_PADCAL_TRANSN(val)            (((val) & 0x1f) << 6)
#define USB_DEVICE_PADCAL_TRIM(val)              (((val) & 0x7) << 12)
    uint8_t  resvd_0x30[214];
    volatile usb_ep_t ep[USB_EP_NUM];
} __attribute__((packed)) usb_device_regs_t;

typedef struct usb_ep_desc_bank
{
    uint32_t addr;
    uint32_t pcksize;
#define USB_EP_DESC_BYTE_COUNT(val)              (((val) & 0x3fff) << 0)
#define USB_EP_DESC_BYTE_COUNT_GET(reg)          (((reg) >> 0) & 0x3fff)
#define USB_EP_DESC_MULTI_PACKET_SIZE(val)       (((val) & 0x3fff) << 14)
#define USB_EP_DESC_MULTI_PACKET_GET(reg)        (((val) >> 14) & 0x3fff)
#define USB_EP_DESC_MULTI_PACKET_MAX             (0x3fff)
#define USB_EP_DESC_SIZE(val)                    (((val) & 0x7) << 28)
#define USB_EP_DESC_SIZE_8                       (0x0)
#define USB_EP_DESC_SIZE_16                      (0x1)
#define USB_EP_DESC_SIZE_32                      (0x2)
#define USB_EP_DESC_SIZE_64                      (0x3)
#define USB_EP_DESC_SIZE_128                     (0x4)
#define USB_EP_DESC_SIZE_256                     (0x5)
#define USB_EP_DESC_SIZE_512                     (0x6)
#define USB_EP_DESC_SIZE_1023                    (0x7)
#define USB_EP_DESC_AUTO_ZLP                     (1 << 31)
    uint16_t extreg;
#define USB_EP_EXTREG_SUBPID(val)                (((val) & 0xf) << 0)
#define USB_EP_EXTREG_VARIABLE(val)              (((val) & 0xfff) << 4)
    uint8_t  status_bk;
#define USB_EP_STATUS_BK_CRCERR                  (1 << 0)
#define USB_EP_STATUS_BK_ERRORFLOW               (1 << 1)
    uint8_t  resvd_0x0b[5];
} __attribute__((packed)) usb_ep_desc_bank_t;

typedef struct usb_ep_desc
{
    volatile usb_ep_desc_bank_t bank[2];
} __attribute__ ((packed)) __attribute__((aligned(4))) usb_ep_desc_t;

#define USB_EP_DESC_BANK_OUT                     0
#define USB_EP_DESC_BANK_IN                      1

#if defined(__AT91SAML21__) || defined(__ATSAMD53__)
#define USB                                      ((volatile usb_device_regs_t *)0x41000000)
#endif /* __AT91SAML21__ || __ATSAMD53__ */


#if defined(__AT91SAMD20__)
#endif /* __AT91SAMD20__ */


int cmd_usb(console_t *console, int argc, char *argv[]);
#define CONSOLE_CMD_USB                          \
    {                                            \
        .cmdstr = "usb",                         \
        .callback = cmd_usb,                     \
        .usage = "  usb show\r\n",               \
        .help =                                  \
            "  USB control and debug.\r\n" \
            "    show       : Show register and ep config.\r\n" \
    }


#endif /* __SAML_USB_H__ */
