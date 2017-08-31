/*
 * saml_reset.h
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

#ifndef __SAML_RESET_H__
#define __SAML_RESET_H__


#include <vectors.h>

#include "saml_sercom.h"


typedef struct reset
{
    uint8_t  rcause;
#define RESET_RCAUSE_POR                         (1 << 0)
#define RESET_RCAUSE_BOD12                       (1 << 1)
#define RESET_RCAUSE_BOD33                       (1 << 2)
#define RESET_RCAUSE_EXT                         (1 << 4)
#define RESET_RCAUSE_WDT                         (1 << 5)
#define RESET_RCAUSE_SYST                        (1 << 6)
#define RESET_RCAUSE_BACKUP                      (1 << 7)
    uint8_t  resvd_0x1;
    uint8_t  bkupexit;
#define RESET_BKUPEXIT_EXTWAKE                   (1 << 0)
#define RESET_BKUPEXIT_RTC                       (1 << 1)
#define RESET_BKUPEXIT_BBPS                      (1 << 2)
    uint8_t  resvd_0x3;
    uint8_t  wkdbconf;
#define RESET_WKDBCONF_2CK32                     (0x1 << 0)
#define RESET_WKDBCONF_3CK32                     (0x2 << 0)
#define RESET_WKDBCONF_32CK32                    (0x3 << 0)
#define RESET_WKDBCONF_512CK32                   (0x4 << 0)
#define RESET_WKDBCONF_4096CK32                  (0x5 << 0)
#define RESET_WKDBCONF_32768CK32                 (0x6 << 0)
    uint8_t  resvd_0x5[3];
    uint16_t wkpol;
#define RESET_WKPOL_ACTIVE_LOW                   (0x0 << 0)
#define RESET_WKPOL_ACTIVE_HIGH                  (0x1 << 0)
    uint16_t resvd_0xa;
    uint16_t wken;
#define RESET_WKEN_DISABLED                      (0x0 << 0)
#define RESET_WKEN_ENABLED                       (0x1 << 0)
    uint16_t resvd_0xe;
    uint16_t wkcause;
#define RESET_WKCAUSE_NOT_ACTIVE                 (0x0 << 0)
#define RESET_WKCAUSE_ACTIVE                     (0x1 << 0)
} __attribute__ ((packed)) reset_t;

#define RESET                                    ((volatile reset_t *)0x40000800)


#define AIRCR_SYSRESETREQ                        (1 << 2)
#define AIRCR_VECTKEY                            (0x05fa << 16)
#define AIRCR                                    ((volatile uint32_t *)0xe000ed0c)


static inline void saml_soft_reset(void)
{
    barrier();
    *AIRCR = (AIRCR_SYSRESETREQ | AIRCR_VECTKEY);
}


#define SRAM_BASE_ADDRESS                        0x20000000
// We'll place the reset config word where the stack pointer is loaded from the vector
// table.  It's no longer used after loading, and is reset during loading
#define RESET_CONFIG                             (*(volatile uint32_t *)SRAM_BASE_ADDRESS)
#define RESET_CONFIG_APPLICATION                 (0x0 << 0)
#define RESET_CONFIG_BOOTLOADER                  (0x1 << 0)


#define CONSOLE_CMD_RESET                        \
    {                                            \
        .cmdstr = "reset",                       \
        .callback = cmd_reset,                   \
        .usage = "  reset < show | app | bootloader >\r\n",       \
        .help =                                  \
            "  Reset device and reset reason information.\r\n" \
            "    show       : Display the last reset reason.\r\n" \
            "    app        : Reset device and enter application.\r\n" \
            "    bootloader : Reset device and enter bootloader.\r\n", \
    }



int cmd_reset(uart_drv_t *uart, int argc, char *argv[]);


#endif /* __SAML_RESET_H__ */
