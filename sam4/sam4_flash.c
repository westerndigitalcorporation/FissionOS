/*
 * sam4_flash.c
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


#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "sam4_flash.h"
#include "sam4_clock.h"
#include "sam4_reset.h"

#include "console.h"

__attribute__ ((section(".sram_function")))
static int eefc_command_issue(volatile eefc_t *effc, const uint8_t cmd, const uint16_t arg)
{
    uint32_t tmp, ret = 0;

    while (!(effc->fsr & EEFC_FSR_FRDY))
        ;

    effc->fcr = EEFC_FCR_FCMD(cmd) | EEFC_FCR_FARG(arg) | EEFC_FCR_KEY;

    while (!((tmp = effc->fsr) & EEFC_FSR_FRDY))
        ;

    if (tmp & EEFC_FSR_FCMDE)
    {
        ret = EEFC_COMMAND_FAILED;
    }

    if (tmp & EEFC_FSR_FLOCKE)
    {
        ret = EEFC_WRITE_FAILED;
    }

    if(tmp & EEFC_FSR_FLOCKE)
    {
        ret = EEFC_PAGE_LOCKED;
    }

    return ret;
}

// Work around buggy compilers that do reads before writes when the read isn't needed.
__attribute__ ((section(".sram_function")))
void eefc_write_address(uint32_t *dst, uint32_t val)
{
    uint32_t addr = (uint32_t)dst;

    asm volatile ("str %0, [%1];"
                  :
                  : "r" (val), "r" (addr)
                  :
        );
}

__attribute__ ((section(".sram_function")))
int eefc_command_issue_reset(volatile eefc_t *eefc, const uint8_t cmd, const uint16_t arg)
{
    int ret;

    ret = eefc_command_issue(eefc, cmd, arg);
    if (ret)
    {
        return ret;
    }

    RESET();

    return 0;
}

int eefc_4k_erase(volatile eefc_t *eefc, uint32_t start_addr)
{
    return eefc_command_issue(eefc, EEFC_FCR_CMD_EPA,
                              (start_addr >> 9) | EEFC_FCR_ARG_8_PAGES);
    return 0;
}

int eefc_write_page(volatile eefc_t *eefc, uint32_t addr, char *buffer)
{
    uint32_t *dst = (uint32_t *)addr;
    uint32_t *src = (uint32_t *)buffer;
    int i;

    // Do a word copy to the target address
    for (i = 0; i < EEFC_PAGE_SIZE / sizeof(uint32_t); i++)
    {
        eefc_write_address(dst++, *src++);
    }

    return eefc_command_issue(eefc, EEFC_FCR_CMD_WP, addr >> EEFC_PAGE_SHIFT);
}

int eefc_desc_read(volatile eefc_t *eefc, eefc_desc_t *desc)
{
    uint32_t *buffer = (uint32_t *)desc;
    int i;

    if (eefc_command_issue(eefc, EEFC_FCR_CMD_GETD, 0) != 0)
    {
        return -1;
    }

    for (i = 0; i < sizeof(*desc) / sizeof(uint32_t); i++)
    {
        buffer[i] = eefc->frr;
    }

    return 0;
}

int eefc_gpnvm_get(uint32_t *gpnvm)
{
    if (eefc_command_issue(EEFC0, EEFC_FCR_CMD_GGPB, 0))
    {
        return -1;
    }

    *gpnvm = EEFC0->frr;

    return 0;
}

int eefc_gpnvm_change(uint8_t command, int bit)
{
    if (eefc_command_issue_reset(EEFC0, command, bit))
    {
        return -1;
    }

    return 0;
}


int cmd_flash(console_t *console, int argc, char *argv[])
{
    eefc_desc_t desc;
    uint32_t gpnvm;

    eefc_gpnvm_get(&gpnvm);
    console_print(console, "GPNVM 0x%08x  %s  Boot: %s  Boot Flash Bank: %s\r\n",
                  gpnvm,
                  gpnvm & (1 << GPNVM_SECURITY_BIT) ? "Protected" : "Not Protected",
                  gpnvm & (1 << GPNVM_BOOT_FLASH_BIT) ? "Flash" : "ROM",
                  gpnvm & (1 << GPNVM_BOOT_BANK_BIT) ? "EFFC1" : "EFFC0");

    eefc_desc_read(EEFC0, &desc);
    console_print(console, "EEFC0 ID 0x%08x   Size %dk   Page Size %d   Planes %d\r\n",
                  desc.fl_id, desc.fl_size / 1024, desc.fl_page_size, desc.fl_nb_plane);

#if defined(__AT91SAM4S__) && defined(__DUAL_BANK_FLASH__)
    eefc_desc_read(EEFC1, &desc);
    console_print(console, "EEFC1 ID 0x%08x   Size %dk   Page Size %d   Planes %d\r\n",
                  desc.fl_id, desc.fl_size / 1024, desc.fl_page_size, desc.fl_nb_plane);
#endif

    return 0;
}

