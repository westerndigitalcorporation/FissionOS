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

#include <console.h>
#include <crc.h>

#include "saml_arch.h"

#include "saml_nvm.h"

#if defined(__AT91SAML21__) || defined(__AT91SAMD20__)

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
    volatile uint32_t *dst = (uint32_t *)addr;

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

int cmd_nvm(console_t *console, int argc, char *argv[])
{
    uint32_t bytes = 0;
    uint8_t tmp;

    if (argc < 2)
    {
        cmd_help_usage(console, argv[0]);
        return 0;
    }

    if (!strcmp(argv[1], "show"))
    {
        console_print(console, "NVM User Row %08x %08x\r\n",
                ((uint32_t *)&NVM_USER_ROW->row)[1],
                ((uint32_t *)&NVM_USER_ROW->row)[0]);

        tmp = NVM_USER_BOOTPROT_GET(NVM_USER_ROW->row);
        if (tmp != 7)
        {
            bytes = (1 << 8) << (7 - tmp);
        }
        console_print(console, "  BOOTPROT      : 0x%x (%d bytes)\r\n", tmp, bytes);

        tmp = NVM_USER_EEPROM_GET(NVM_USER_ROW->row);
        bytes = 0;
        if (tmp != 7)
        {
            bytes = (1 << 7) << (7 - tmp);
        }
        console_print(console, "  EEPROM        : 0x%x (%d bytes)\r\n\n", tmp, bytes);

        return 0;
    }
    else if ((argc == 3) && !strcmp(argv[1], "bootprot"))
    {
        nvm_user_t user =
        {
            .row = NVM_USER_BOOTPROT_SET(NVM_USER_ROW->row, strtoul(argv[2], NULL, 0)),
        };

        nvm_user_row_write(&user);

        console_print(console, "BOOTPROT Updated\r\n");

        return 0;
    }
    else if ((argc == 3) && !strcmp(argv[1], "eeprom"))
    {
        nvm_user_t user =
        {
            .row = NVM_USER_EEPROM_SET(NVM_USER_ROW->row, strtoul(argv[2], NULL, 0)),
        };

        nvm_user_row_write(&user);

        console_print(console, "EEPROM Updated\r\n");

        return 0;
    }

    cmd_help_usage(console, argv[0]);

    return 0;
}

#endif /* defined(__AT91SAML21__) || defined(__AT91SAMD20__) */

#if defined(__ATSAMD53__)

void nvm_cache_disable(void)
{
    uint16_t ctrla = NVMCTRL->ctrla;

    ctrla |= (NVMCTRL_CTRLA_CACHEDIS0 | NVMCTRL_CTRLA_CACHEDIS1);

    write16(&NVMCTRL->ctrla, ctrla);
}

void nvm_cache_enable(void)
{
    uint16_t ctrla = NVMCTRL->ctrla;

    ctrla &= ~(NVMCTRL_CTRLA_CACHEDIS0 | NVMCTRL_CTRLA_CACHEDIS1);

    write16(&NVMCTRL->ctrla, ctrla);
}

int nvm_page_count(void)
{
    int nvm_pages = (NVMCTRL->param >> NVMCTRL_PARAM_NVMP_SHIFT) &
                     NVMCTRL_PARAM_NVMP_MASK;

    return nvm_pages;
}

int nvm_page_size(void)
{
    int page_size_val = (NVMCTRL->param >> NVMCTRL_PARAM_PSZ_SHIFT) &
                         NVMCTRL_PARAM_PSZ_MASK;

    return (0x8 << (page_size_val));

}

void nvm_busy_wait(void)
{
    while (!(NVMCTRL->status & NVMCTRL_STATUS_READY))
        ;
}

void nvm_command(uint32_t cmd)
{
    write16(&NVMCTRL->ctrlb, NVMCTRL_CTRLB_CMD(cmd) | NVMCTRL_CTRLB_CMDEX);
    nvm_busy_wait();
}

void nvm_block_erase(uint32_t addr)
{
    nvm_command(NVMCTRL_CTRLB_CMD_EB);
}

void nvm_switch_bank(void)
{
    nvm_command(NVMCTRL_CTRLB_CMD_BKSWRST);
}

void nvm_page_write(void)
{
    nvm_command(NVMCTRL_CTRLB_CMD_WP);
}

uint32_t nvm_crc32(uint32_t addr, uint32_t len)
{
    uint32_t page_size = nvm_page_size();
    uint32_t page_count = nvm_page_count();
    uint32_t offset = (page_size * page_count) / 2;

    // Set maximum boundary in case len is corrupt
    if (len > offset)
    {
        len = offset;
    }

    return crc32((uint8_t *)(addr + offset), len);
}

// Make sure that addr and length are 32-bit aligned when using this
// routine.  The data address can be byte aligned.
void nvm_write(uint32_t addr, uint8_t *data, uint32_t len)
{
    uint32_t page_size = nvm_page_size();
    uint32_t page_count = nvm_page_count();
    uint32_t offset = (page_size * page_count) / 2;
    volatile uint32_t *dst = (uint32_t *)(addr + offset);

    // Last write?
    if (!len && (addr & NVM_PAGE_MASK)) {
        nvm_page_write();
        return;
    }

    len = len / sizeof(uint32_t);
    while (len--)
    {
        // Copy from potentially unaligned region as little endian
        uint32_t src = (data[0] << 0) |
                       (data[1] << 8) |
                       (data[2] << 16) |
                       (data[3] << 24);
        *dst = src;

        // Erase at the beginning of a erase block
        if (!((uint32_t)dst & NVM_ERASE_MASK))
        {
            nvm_block_erase((uint32_t)dst);
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

int nvm_active_bank(void)
{
    return NVMCTRL->status & NVMCTRL_STATUS_AFIRST ? 1 : 0;
}

uint32_t nvm_bank_offset(void)
{
    int page_size = nvm_page_size();
    int page_count = nvm_page_count();

    return (page_count * page_size) >> 1;
}

void nvm_pagebuf_clear(void)
{
    volatile uint16_t ctrlb;

    ctrlb = NVMCTRL_CTRLB_CMD_PBC | NVMCTRL_CTRLB_CMDEX;
    NVMCTRL->ctrlb = ctrlb;

    nvm_busy_wait();
}

int cmd_nvm(console_t *console, int argc, char *argv[])
{
    int page_size = nvm_page_size();
    int page_mask = page_size - 1;
    int page_count = nvm_page_count();

    if (argc < 2)
    {
        cmd_help_usage(console, argv[0]);
        return 0;
    }

    if (!strcmp(argv[1], "show"))
    {
        console_print(console, "  param            : 0x%08x\r\n", NVMCTRL->param);
        console_print(console, "  page size        : %d bytes\r\n", page_size);
        console_print(console, "  page mask        : 0x%08x\r\n", page_mask);
        console_print(console, "  page count       : %d\r\n", page_count);
        console_print(console, "  total size       : %dkB\r\n", (page_count * page_size) / 1024);
        console_print(console, "  active bank      : %d\r\n", nvm_active_bank());
        console_print(console, "  bank offset      : %08x\r\n", nvm_bank_offset());

        return 0;
    } else if (!strcmp(argv[1], "swbk")) {
        console_print(console, "Switching active bank\r\n");
        nvm_switch_bank();
        while(1);
    }

    cmd_help_usage(console, argv[0]);

    return 0;
}

#endif /* defined(__ATSAMD53__) */
