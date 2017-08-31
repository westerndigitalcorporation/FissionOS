/*
 * sam4_watchdog.h
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


#ifndef __SAM4_WATCHDOG_H__
#define __SAM4_WATCHDOG_H__

typedef struct wdt
{
    uint32_t cr;
#define WDT_CR_WDRSTT          (1 << 0)
#define WDT_CR_KEY             (0xa5 << 24)
    uint32_t mr;
#define WDT_MR_WDV(val)        ((val & 0x0fff) << 0)
#define WDT_MR_WDFIEN          (1 << 12)
#define WDT_MR_WDRSTEN         (1 << 13)
#define WDT_MR_WDRPROC         (1 << 14)
#define WDT_MR_WDDIS           (1 << 15)
#define WDT_MR_WDD(val)        ((val & 0x0fff) << 16)
#define WDT_MR_WDDBGHLT        (1 << 28)
#define WDT_MR_WDIDLEHLT       (1 << 29)
    uint32_t sr;
#define WDT_SR_WDUNF           (1 << 0)
#define WDT_SR_WDERR           (1 << 1)
} __attribute__ ((packed)) wdt_t;

#if defined(__AT91SAM4S__)
#define WDT                    ((volatile wdt_t *)0x400e1450)
#elif defined(__AT91SAM4E__)
#define WDT                    ((volatile wdt_t *)0x400e1850)
#endif

void wdt_disable(void);

#endif /* __SAM4_WATCHDOG_H__ */
