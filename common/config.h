/*
 * config.h
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


#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "flash.h"
#include "console.h"


#define CONSOLE_CMD_CONFIG                       \
    {                                            \
        .cmdstr = "config",                      \
        .callback = cmd_config,                     \
        .usage = "  config\r\n", \
        .help =  "  Display Firmware Flash Configuration.\r\n",  \
    }


#define CONFIG_MAGIC                             0x12098723
typedef struct config_descriptor
{
    uint32_t magic;
    uint32_t boot_bank;
    uint32_t eefc0_image_len;
    uint32_t eefc0_image_crc;
    uint32_t eefc1_image_len;
    uint32_t eefc1_image_crc;
    uint32_t crc;
} __attribute__ ((packed)) config_desc_t;


int config_load(config_desc_t *config);
int config_save(config_desc_t *config);
void config_new(config_desc_t *config);

void config_image_switch(void);
int config_image_update(uint32_t boot_bank, uint32_t image_len, uint32_t image_crc);

int cmd_config(uart_drv_t *uart, int argc, char *argv[]);

#endif /* __CONFIG_H__ */
