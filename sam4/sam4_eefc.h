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


#ifndef __SAM4_EEFC_H__
#define __SAM4_EEFC_H__


#define GPNVM_SECURITY_BIT                      0
#define GPNVM_BOOT_FLASH_BIT                    1
#define GPNVM_BOOT_BANK_BIT                     2

#define EEFC_PAGE_SHIFT                         9
#define EEFC_PAGE_SIZE                          (1 << EEFC_PAGE_SHIFT)
#define EEFC_PAGE_OFFSET_MASK                   (EEFC_PAGE_SIZE - 1)
#define EEFC_PAGE_MASK                          ~(EEFC_PAGE_SIZE - 1)

#define EEFC_MIN_ERASE_SIZE                     0xfff

#define EEFC0_BASE_ADDR                         0x400000
#define EEFC1_BASE_ADDR                         0x500000

typedef struct eefc
{
    uint32_t fmr;
#define EEFC_FMR_FRDY                           (1 << 0)
#define EEFC_FMR_FWS(val)                       ((val & 0xf) << 8)
#define EEFC_FMR_SCOD                           (1 << 16)
#define EEFC_FMR_FAM                            (1 << 24)
#define EEFC_FMR_CLOE                           (1 << 26)
    uint32_t fcr;
#define EEFC_FCR_CMD_GETD                       0x00
#define EEFC_FCR_CMD_WP                         0x01
#define EEFC_FCR_CMD_WPL                        0x02
#define EEFC_FCR_CMD_EWP                        0x03
#define EEFC_FCR_CMD_EWPL                       0x04
#define EEFC_FCR_CMD_EA                         0x05
#define EEFC_FCR_CMD_EPA                        0x07
#define EEFC_FCR_CMD_SLB                        0x08
#define EEFC_FCR_CMD_CLB                        0x09
#define EEFC_FCR_CMD_GLB                        0x0a
#define EEFC_FCR_CMD_SGPB                       0x0b
#define EEFC_FCR_CMD_CGPB                       0x0c
#define EEFC_FCR_CMD_GGPB                       0x0d
#define EEFC_FCR_CMD_STUI                       0x0e
#define EEFC_FCR_CMD_SPUI                       0x0f
#define EEFC_FCR_CMD_GCALB                      0x10
#define EEFC_FCR_CMD_ES                         0x11
#define EEFC_FCR_CMD_WUS                        0x12
#define EEFC_FCR_CMD_EUS                        0x13
#define EEFC_FCR_CMD_STUS                       0x14
#define EEFC_FCR_CMD_SPUS                       0x15
#define EEFC_FCR_FCMD(val)                      ((val & 0xff) << 0)
#define EEFC_FCR_ARG_4_PAGES                    (0 << 0)
#define EEFC_FCR_ARG_8_PAGES                    (1 << 0)
#define EEFC_FCR_ARG_16_PAGES                   (2 << 0)
#define EEFC_FCR_ARG_32_PAGES                   (3 << 0)
#define EEFC_FCR_ARG_PAGES_MASK                 0x3
#define EEFC_FCR_FARG(val)                      ((val & 0xffffUL) << 8)
#define EEFC_FCR_KEY                            (0x5aUL << 24)
    uint32_t fsr;
#define EEFC_FSR_FRDY                           (1 << 0)
#define EEFC_FSR_FCMDE                          (1 << 1)
#define EEFC_FSR_FLOCKE                         (1 << 2)
#define EEFC_FSR_FLERR                          (1 << 3)
    uint32_t frr;
} __attribute__ ((packed)) eefc_t;

#define EEFC0_ADDR                              0x400e0a00
#define EEFC0                                   ((volatile eefc_t *)EEFC0_ADDR)
#if defined(__DUAL_BANK_FLASH__)
#define EEFC1_ADDR                              0x400e0c00
#define EEFC1                                   ((volatile eefc_t *)EEFC1_ADDR)
#endif /* __DUAL_BANK_FLASH__ */

#define EEFC_WRITE_FAILED                        -1
#define EEFC_PAGE_LOCKED                         -2
#define EEFC_COMMAND_FAILED                      -3


#endif /* __SAM4_EEFC_H__ */
