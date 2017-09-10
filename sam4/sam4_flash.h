/*
 * sam4_flash.h
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


#ifndef __SAM4S_FLASH_H__
#define __SAM4S_FLASH_H__

#include <console.h>

#include "sam4_eefc.h"

typedef struct eefc_desc
{
    uint32_t fl_id;
    uint32_t fl_size;
    uint32_t fl_page_size;
    uint32_t fl_nb_plane;
} __attribute__ ((packed)) eefc_desc_t;

int eefc_desc_read(volatile eefc_t *eefc, eefc_desc_t *desc);


int eefc_gpnvm_get(uint32_t *gpnvm);
int eefc_gpnvm_change(uint8_t command, int bit);

static inline int eefc_gpnvm_set_reset(int bit)
{
    return eefc_gpnvm_change(EEFC_FCR_CMD_SGPB, bit);
}

static inline int eefc_gpnvm_clear_reset(int bit)
{
    return eefc_gpnvm_change(EEFC_FCR_CMD_CGPB, bit);
}

static inline int eefc_boot_bank(void)
{
    uint32_t gpnvm;

    eefc_gpnvm_get(&gpnvm);
    return gpnvm & (1 << GPNVM_BOOT_BANK_BIT) ? 1 : 0;
}

int eefc_4k_erase(volatile eefc_t *eefc, uint32_t start_addr);
int eefc_write_page(volatile eefc_t *eefc, uint32_t addr, char *buffer);

int cmd_flash(console_t *console, int argc, char *argv[]);


#endif  /* __SAM4S_FLASH_H__ */
