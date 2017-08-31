/*
 * cortexm_debug.h
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


#ifndef __CORTEXM_DEBUG_H__
#define __CORTEXM_DEBUG_H__

#define HW_BP_MAX                                4

/* Cortex-M3/4 Application Interrupt and Reset Control Register */
#define SCB_AIRCR                                0xe000ed0c
#define   SCB_AIRCR_VECTRESET                    (1 << 0)
#define   SCB_AIRCR_VECTCLRACTIVE                (1 << 1)
#define   SCB_AIRCR_SYSRESETREQ                  (1 << 2)
#define   SCB_AIRCR_PRIGROUP(val)                ((val & 0x7) << 8)
#define   SCB_AIRCR_ENDIANNESS                   (1 << 15)
#define   SCB_AIRCR_VECTKEY                      (0x05faUL << 16)

/* Cortex-M3/4 Core Debug Registers */
#define CORTEX_DHCSR                             0xe000edf0            //*< Core/Status register */
#define   CORTEX_DHCSR_DBGKEY                    (0xa05fUL << 16)
#define   CORTEX_DHCSR_DEBUGEN                   (1 << 0)
#define   CORTEX_DHCSR_HALT                      (1 << 1)
#define   CORTEX_DHCSR_STEP                      (1 << 2)

#define CORTEX_DCRSR                             0xe000edf4            //*< Register select register */
#define   CORTEX_DCRSR_REGSEL(val)               ((val & 0x1f) << 0)
#define     CORTEX_REG_R(val)                    (val & 0xf)
#define     CORTEX_REG_SP                        0x0d
#define     CORTEX_REG_LR                        0x0e
#define     CORTEX_REG_PC                        0x0f
#define     CORTEX_REG_xPSR                      0x10
#define     CORTEX_REG_MSP                       0x11
#define     CORTEX_REG_PSP                       0x12
#define     CORTEX_MAX_REGS                      (CORTEX_REG_PSP + 1)
#define   CORTEX_DCRSR_WRITE                     (1UL << 16)
#define CORTEX_DCRDR                             0xe000edf8

#define SWD_CORTEX_HALT(swd_instance)            swd_mem_write(swd_instance, CORTEX_DHCSR, \
                                                               CORTEX_DHCSR_DBGKEY | CORTEX_DHCSR_DEBUGEN | CORTEX_DHCSR_HALT)
#define SWD_CORTEX_RESUME(swd_instance)          swd_mem_write(swd_instance, CORTEX_DHCSR, \
                                                               CORTEX_DHCSR_DBGKEY | CORTEX_DHCSR_DEBUGEN)
#define SWD_CORTEX_STEP(swd_instance)            swd_mem_write(swd_instance, CORTEX_DHCSR, \
                                                               CORTEX_DHCSR_DBGKEY | CORTEX_DHCSR_DEBUGEN | CORTEX_DHCSR_STEP)
#define SWD_CORTEX_RESET(swd_instance)           swd_mem_write(swd_instance, SCB_AIRCR, SCB_AIRCR_SYSRESETREQ | \
                                                               SCB_AIRCR_VECTKEY)

typedef struct cortex_fp
{
    uint32_t ctrl;
#define CORTEX_FP_CTRL_ENABLE                    (1 << 0)
#define CORTEX_FP_CTRL_KEY                       (1 << 1)
#define CORTEX_FP_NUM_CODE1(val)                 ((val & 0xf) << 4)
#define CORTEX_FP_NUM_LIT(val)                   ((val & 0xf) << 8)
#define CORTEX_FP_NUM_CODE2(val)                 ((val & 0x3) << 12)
    uint32_t remap;
    uint32_t comp[8];
#define CORTEX_FP_COMP_ENABLE                    (1 << 0)
#define CORTEX_FP_COMP_COMP(val)                 (val & 0x1ffffffcUL)
#define CORTEX_FP_COMP_REPLACE(val)              ((val & 0x3UL) << 30)
#define CORTEX_FP_COMP_REPLACE_REMAP             0x0
#define CORTEX_FP_COMP_REPLACE_BKPT_LOWER        0x1
#define CORTEX_FP_COMP_REPLACE_BKPT_UPPER        0x2
#define CORTEX_FP_COMP_REPLACE_BKPT_BOTH         0x3
} __attribute__ ((packed)) cortex_fp_t;

#define CORTEX_FP_ADDR                           0xe0002000
#define CORTEX_FP                                ((volatile cortex_fp_t *)CORTEX_FP_ADDR)

#define SWD_CORTEX_BP_ENABLE(swd_instance)       swd_mem_write(swd_instance, CORTEX_FP_ADDR + offsetof(cortex_fp_t, ctrl), \
                                                               CORTEX_FP_CTRL_ENABLE | CORTEX_FP_CTRL_KEY)
#define SWD_CORTEX_BP_DISABLE(swd_instance)      swd_mem_write(swd_instance, (uint32_t)&CORTEX_FP->ctrl, \
                                                               CORTEX_FP_CTRL_KEY)
#define SWD_CORTEX_BP_SET(swd_instance, bp, addr, replace)  swd_mem_write(swd_instance, \
                                                                          CORTEX_FP_ADDR + (uint32_t)offsetof(cortex_fp_t, comp[bp]), \
                                                                          CORTEX_FP_COMP_ENABLE | \
                                                                          CORTEX_FP_COMP_COMP(addr) | \
                                                                          CORTEX_FP_COMP_REPLACE(replace))
#define SWD_CORTEX_BP_CLEAR(swd_instance, bp)    swd_mem_write(swd_instance, CORTEX_FP_ADDR + (uint32_t)offsetof(cortex_fp_t, comp[bp]), 0)

#endif /* __CORTEXM_DEBUG_H__ */
