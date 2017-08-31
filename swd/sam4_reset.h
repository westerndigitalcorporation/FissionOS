/*
 * sam4_acc.c
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

#ifndef __SAM4_RESET_H__
#define __SAM4_RESET_H__

typedef struct rstc
{
    uint32_t cr;
#define RSTC_CR_PROCRST                          (1 << 0)
#define RSTC_CR_PERRST                           (1 << 2)
#define RSTC_CR_EXTRST                           (1 << 3)
#define RSTC_CR_KEY                              (0xa5UL << 24)
    uint32_t sr;
#define RSTC_SR_URSTS                            (1 << 0)
#define RSTC_SR_RSTTYPE(val)                     ((val >> 8) & 0x7)
#define RSTC_RSTTYPE_POWERON                     0
#define RSTC_RSTTYPE_BACKUP                      1
#define RSTC_RSTTYPE_WATCHDOG                    2
#define RSTC_RSTTYPE_SOFTWARE                    3
#define RSTC_RSTTYPE_EXTERNAL                    4
#define RSTC_SR_NRSTL                            (1 << 16)
#define RSTC_SR_SRCMP                            (1 << 17)
    uint32_t mr;
} __attribute__((packed)) rstc_t;

#define RSTC_AT91SAM4S_ADDR                      0x400e1400
#define RSTC_AT91SAM4E_ADDR                      0x400e1800

#if defined(__AT91SAM4S__)
#define RSTC                                     ((volatile rstc_t *)RSTC_AT91SAM4S_ADDR)
#elif defined(__AT91SAM4E__)
#define RSTC                                     ((volatile rstc_t *)RSTC_AT91SAM4E_ADDR)
#endif

#define RESET()                                  do {                                              \
                                                     RSTC->cr = RSTC_CR_PROCRST | RSTC_CR_PERRST | \
                                                                RSTC_CR_KEY;                       \
                                                 } while(1)

#endif /* __SAM4_RESET_H__ */
