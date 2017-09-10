/*
 * config.c
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

#include "systick.h"

#include "sam4_clock.h"
#include "sam4_reset.h"

#include "workqueue.h"
#include "flash.h"
#include "config.h"
#include "crc.h"

#include "console.h"


int config_load(config_desc_t *config)
{
#if defined(__DUAL_BANK_FLASH__)
    uint32_t base_addr = eefc_boot_bank() ? EEFC1_BASE_ADDR : EEFC0_BASE_ADDR;
#else  /* __DUAL_BANK_FLASH__ */
    uint32_t base_addr = EEFC0_BASE_ADDR;
#endif /* __DUAL_BANK_FLASH__ */
    uint32_t crc;

    memcpy(config, (uint8_t *)(base_addr + flash_desc.config_offset),
           sizeof(*config));

    if (config->magic != CONFIG_MAGIC)
    {
        return -1;
    }

    crc = config->crc;
    config->crc = 0;

    config->crc = crc32((uint8_t *)config, sizeof(*config));
    if (crc != config->crc)
    {
        return -1;
    }

    return 0;
}

int config_save(config_desc_t *config)
{
#if defined(__DUAL_BANK_FLASH__)
    uint32_t base_addr = eefc_boot_bank() ? EEFC1_BASE_ADDR : EEFC0_BASE_ADDR;
#else  /* __DUAL_BANK_FLASH__ */
    uint32_t base_addr = EEFC0_BASE_ADDR;
#endif /* __DUAL_BANK_FLASH__ */
    uint32_t dst = (uint32_t)flash_desc.config_offset + base_addr;
    uint8_t buffer[EEFC_PAGE_SIZE];
    int ret;

    config->crc = 0;
    config->crc = crc32((uint8_t *)config, sizeof(*config));

    ret = eefc_4k_erase(flash_desc.config_flash_bank, dst);
    if (ret)
    {
        return ret;
    }

    memset(buffer, 0xff, EEFC_PAGE_SIZE);
    memcpy(buffer, config, sizeof(*config));

    ret = eefc_write_page(flash_desc.config_flash_bank, dst, (char *)buffer);

    return ret;
}

void config_new(config_desc_t *config)
{
    memset(config, 0, sizeof(*config));
    config->magic = CONFIG_MAGIC;
}

#if defined(__DUAL_BANK_FLASH__)
void config_image_switch(void)
{
    uint32_t boot_bank = eefc_boot_bank();
    uint32_t eefc0_addr = boot_bank ? EEFC1_BASE_ADDR : EEFC0_BASE_ADDR;
    uint32_t eefc1_addr = boot_bank ? EEFC0_BASE_ADDR : EEFC1_BASE_ADDR;
    config_desc_t config;

    if (config_load(&config))
    {
        return;
    }

    if (config.boot_bank != boot_bank)
    {
        if (config.boot_bank)
        {
            uint32_t crc = crc32((uint8_t *)eefc1_addr, config.eefc1_image_len);

            if (crc == config.eefc1_image_crc)
            {
                eefc_gpnvm_set_reset(GPNVM_BOOT_BANK_BIT);
            }
        }
        else
        {
            uint32_t crc = crc32((uint8_t *)eefc0_addr, config.eefc0_image_len);

            if (crc == config.eefc0_image_crc)
            {
                eefc_gpnvm_clear_reset(GPNVM_BOOT_BANK_BIT);
            }
        }
    }
}
#endif /* __DUAL_BANK_FLASH__ */

int config_image_update(uint32_t boot_bank, uint32_t image_len, uint32_t image_crc)
{
    config_desc_t config;

    if (config_load(&config))
    {
        config_new(&config);
    }

    if (boot_bank)
    {
        config.eefc1_image_len = image_len;
        config.eefc1_image_crc = image_crc;
    }
    else
    {
        config.eefc0_image_len = image_len;
        config.eefc0_image_crc = image_crc;
    }

    config.boot_bank = boot_bank;

    return config_save(&config);
}

int cmd_config(console_t *console, int argc, char *argv[])
{
#if defined(__DUAL_BANK_FLASH__)
    uint32_t boot_bank = eefc_boot_bank();
    uint32_t eefc0_addr = boot_bank ? EEFC1_BASE_ADDR : EEFC0_BASE_ADDR;
    uint32_t eefc1_addr = boot_bank ? EEFC0_BASE_ADDR : EEFC1_BASE_ADDR;
#else  /* __DUAL_BANK_FLASH__ */
    uint32_t boot_bank = 0;
    uint32_t eefc0_addr = EEFC0_BASE_ADDR;
#endif /* __DUAL_BANK_FLASH__ */
    config_desc_t config;
    uint32_t crc;

    console_print(console, "\nRunning Bank: %s\r\n\n", boot_bank ? "EEFC1" : "EEFC0");
    if (config_load(&config))
    {
        console_print(console, "No Configuration Found\r\n");
        return 0;
    }

    console_print(console, "Config Descriptor\r\n");
#if defined(__DUAL_BANK_FLASH__)
    console_print(console, "  Next Boot: %s\r\n", config.boot_bank ? "EFFC1" : "EFFC0");
#endif /* __DUAL_BANK_FLASH__ */
    crc = crc32((uint8_t *)eefc0_addr, config.eefc0_image_len);
    console_print(console, "  EEFC0 Image CRC: %08x Len: %9d (%s)\r\n",
                  config.eefc0_image_crc,
                  config.eefc0_image_len,
                  crc && (crc == config.eefc0_image_crc) ? "Valid" : "Invalid");
#if defined(__DUAL_BANK_FLASH__)
    crc = crc32((uint8_t *)eefc1_addr, config.eefc1_image_len);
    console_print(console, "  EEFC1 Image CRC: %08x Len: %9d (%s)\r\n\n",
                  config.eefc1_image_crc,
                  config.eefc1_image_len,
                  crc && (crc == config.eefc1_image_crc) ? "Valid" : "Invalid");
#endif /* __DUAL_BANK_FLASH__ */

    return 0;
}


