/*
 * saml_nvm.c
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
#include <string.h>
#include <stdlib.h>

#include "saml_sercom.h"

#include <console.h>

#include "saml_nvm.h"


static void nvm_busy_wait(void)
{
    while (!(NVMCTRL->intflag & NVMCTRL_INTFLAG_READY))
        ;
}

static void nvm_row_erase(void)
{
    NVMCTRL->ctrla = NVMCTRL_CTRLA_CMD_ER | NVMCTRL_CTRLA_CMDEX;
    nvm_busy_wait();
}

static void nvm_page_write(void)
{
    NVMCTRL->ctrla = NVMCTRL_CTRLA_CMD_WP | NVMCTRL_CTRLA_CMDEX;
    nvm_busy_wait();
}

// Make sure that addr and length are 32-bit aligned when using this
// routine.  The data address can be byte aligned.
void nvm_write(uint32_t addr, uint8_t *data, uint32_t len)
{
    uint32_t *dst = (uint32_t *)addr;

    len = len / sizeof(uint32_t);
    while (len--)
    {
        // Copy from potentially unaligned region as little endian
        uint32_t src = (data[0] << 0) |
                       (data[1] << 8) |
                       (data[2] << 16) |
                       (data[3] << 24);

        *dst = src;

        // Erase at the beginning of a row
        if (!((uint32_t)dst & NVM_ROW_MASK))
        {
            nvm_row_erase();
        }

        // Write at the end of the page
        if (((uint32_t)dst & NVM_PAGE_MASK) == (NVM_PAGE_SIZE - sizeof(uint32_t)))
        {
            nvm_page_write();
        }

        data += sizeof(uint32_t);
        dst++;
    }
}


static void nvm_pagebuf_clear(void)
{
    NVMCTRL->ctrla = NVMCTRL_CTRLA_CMD_PBC | NVMCTRL_CTRLA_CMDEX;
    nvm_busy_wait();
}

static int nvm_auxpage_erase(void)
{
    NVMCTRL->ctrla = NVMCTRL_CTRLA_CMD_EAR | NVMCTRL_CTRLA_CMDEX;
    nvm_busy_wait();

    return 0;
}

static int nvm_auxpage_write(void)
{
    NVMCTRL->ctrla = NVMCTRL_CTRLA_CMD_WAP | NVMCTRL_CTRLA_CMDEX;
    nvm_busy_wait();

    return 0;
}

static int nvm_user_row_write(nvm_user_t *nvmuser)
{
    uint32_t i;

    nvm_pagebuf_clear();

    // Copy the user row into flash pagebuffer
    for (i = 0; i < (sizeof(*nvmuser) / sizeof(uint32_t)); i++)
    {
        ((uint32_t *)NVM_USER_ROW)[i] = ((uint32_t *)nvmuser)[i];
    }

    nvm_auxpage_erase();
    nvm_auxpage_write();

    return 0;
}

int cmd_nvm(uart_drv_t *uart, int argc, char *argv[])
{
    uint32_t bytes = 0;
    uint8_t tmp;

    if (argc < 2)
    {
        cmd_help_usage(uart, argv[0]);
        return 0;
    }

    if (!strcmp(argv[1], "show"))
    {
        console_print("NVM User Row %08x %08x\r\n",
                ((uint32_t *)&NVM_USER_ROW->row)[1],
                ((uint32_t *)&NVM_USER_ROW->row)[0]);

        tmp = NVM_USER_BOOTPROT_GET(NVM_USER_ROW->row);
        if (tmp != 7)
        {
            bytes = (1 << 8) << (7 - tmp);
        }
        console_print("  BOOTPROT      : 0x%x (%d bytes)\r\n", tmp, bytes);

        tmp = NVM_USER_EEPROM_GET(NVM_USER_ROW->row);
        bytes = 0;
        if (tmp != 7)
        {
            bytes = (1 << 7) << (7 - tmp);
        }
        console_print("  EEPROM        : 0x%x (%d bytes)\r\n\n", tmp, bytes);

        return 0;
    }
    else if ((argc == 3) && !strcmp(argv[1], "bootprot"))
    {
        nvm_user_t user =
        {
            .row = NVM_USER_BOOTPROT_SET(NVM_USER_ROW->row, strtoul(argv[2], NULL, 0)),
        };

        nvm_user_row_write(&user);

        console_print("BOOTPROT Updated\r\n");

        return 0;
    }
    else if ((argc == 3) && !strcmp(argv[1], "eeprom"))
    {
        nvm_user_t user =
        {
            .row = NVM_USER_EEPROM_SET(NVM_USER_ROW->row, strtoul(argv[2], NULL, 0)),
        };

        nvm_user_row_write(&user);

        console_print("EEPROM Updated\r\n");

        return 0;
    }

    cmd_help_usage(uart, argv[0]);

    return 0;
}

