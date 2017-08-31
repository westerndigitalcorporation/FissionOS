/*
 * at91sam4e16.h
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


#ifndef __AT91SAM4E16_H__
#define __AT91SAM4E16_H__

#include "sam4_flash.h"

#define FLASH_BANK_SIZE                          0x100000

/*
 * Our reserved configuration space is 4k and lives at the end of flash bank 0.
 */
#define CONFIG_ERASE_PAGES                       8
#define CONFIG_FLASH_BANK                        EEFC0
#define CONFIG_FLASH_BANK_SIZE                   FLASH_BANK_SIZE
#define CONFIG_FLASH_OFFSET                      (CONFIG_FLASH_BANK_SIZE - \
                                                  (CONFIG_ERASE_PAGES * EEFC_PAGE_SIZE))

/*
 * Make sure the flash image won't overlap the MID/Config regions
 */
#define FLASH_IMAGE_MAX_SIZE                     (FLASH_BANK_SIZE - (8 * EEFC_PAGE_SIZE))

#endif /* __AT91SAM4E16_H__ */
