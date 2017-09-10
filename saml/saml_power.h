/*
 * saml_clocks.h
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


#ifndef __SAML_POWER_H__
#define __SAML_POWER_H__

#if defined(__AT91SAML21__)
typedef struct pm
{
    uint8_t  ctrla;
#define PM_CTRLA_IORET                           (1 << 2)
    uint8_t  sleepcfg;
#define PM_SLEEPCFG_SLEEPMODE(val)               (((val) & 0x7) << 0)
    uint8_t  plcfg;
#define PM_PLCFG_PLSEL(val)                      (((val) & 0x3) << 0)
#define PM_PLCFG_PLDIS                           (1 << 7)
    uint8_t  resvd_0x03;
    uint8_t  intenclr;
#define PM_INTENCLR_PLRDY                        (1 << 0)
    uint8_t  intenset;
#define PM_INTENSET_PLRDY                        (1 << 0)
    uint8_t  intflag;
#define PM_INTFLAG_PLRDY                         (1 << 0)
    uint8_t  resvd_0x07;
    uint16_t stdbycfg;
#define PM_STDBYCFG_PDCFG(val)                   (((val) & 0x3) << 0)
#define PM_STDBYCFG_DPGPD0                       (1 << 4)
#define PM_STDBYCFG_DPGPD1                       (1 << 5)
#define PM_STDBYCFG_VREGSMOD(val)                (((val) & 0x3) << 6)
#define PM_STDBYCFG_LINKPD(val)                  (((val) & 0x3) << 8)
#define PM_STDBYCFG_BBIASHS(val)                 (((val) & 0x3) << 10)
#define PM_STDBYCFG_BBIASLP(val)                 (((val) & 0x3) << 12)
} __attribute__((packed)) pm_t;

#define PM_BASE                                  0x40000000
#define PM                                       ((volatile pm_t *)PM_BASE)

#endif /* __AT91SAML21__ */

typedef struct supc
{
    uint32_t intenclr;
#define SUPC_INTENCLR_BOD33RDY                   (1 << 0)
#define SUPC_INTENCLR_BOD33DET                   (1 << 1)
#define SUPC_INTENCLR_BOD33SRDY                  (1 << 2)
#define SUPC_INTENCLR_BOD12RDY                   (1 << 3)
#define SUPC_INTENCLR_BOD12DET                   (1 << 4)
#define SUPC_INTENCLR_BOD12SRDY                  (1 << 5)
#define SUPC_INTENCLR_VREGRDY                    (1 << 8)
#define SUPC_INTENCLR_APWSRDY                    (1 << 9)
#define SUPC_INTENCLR_VCORERDY                   (1 << 10)
    uint32_t intenset;
#define SUPC_INTENSET_BOD33RDY                   (1 << 0)
#define SUPC_INTENSET_BOD33DET                   (1 << 1)
#define SUPC_INTENSET_BOD33SRDY                  (1 << 2)
#define SUPC_INTENSET_BOD12RDY                   (1 << 3)
#define SUPC_INTENSET_BOD12DET                   (1 << 4)
#define SUPC_INTENSET_BOD12SRDY                  (1 << 5)
#define SUPC_INTENSET_VREGRDY                    (1 << 8)
#define SUPC_INTENSET_APWSRDY                    (1 << 9)
#define SUPC_INTENSET_VCORERDY                   (1 << 10)
    uint32_t intflag;
#define SUPC_INTFLAG_BOD33RDY                    (1 << 0)
#define SUPC_INTFLAG_BOD33DET                    (1 << 1)
#define SUPC_INTFLAG_BOD33SRDY                   (1 << 2)
#define SUPC_INTFLAG_BOD12RDY                    (1 << 3)
#define SUPC_INTFLAG_BOD12DET                    (1 << 4)
#define SUPC_INTFLAG_BOD12SRDY                   (1 << 5)
#define SUPC_INTFLAG_VREGRDY                     (1 << 8)
#define SUPC_INTFLAG_APWSRDY                     (1 << 9)
#define SUPC_INTFLAG_VCORERDY                    (1 << 10)
    uint32_t status;
#define SUPC_STATUS_BOD33RDY                     (1 << 0)
#define SUPC_STATUS_BOD33DET                     (1 << 1)
#define SUPC_STATUS_BOD33SRDY                    (1 << 2)
#define SUPC_STATUS_BOD12RDY                     (1 << 3)
#define SUPC_STATUS_BOD12DET                     (1 << 4)
#define SUPC_STATUS_BOD12SRDY                    (1 << 5)
#define SUPC_STATUS_VREGRDY                      (1 << 8)
#define SUPC_STATUS_APWSRDY                      (1 << 9)
#define SUPC_STATUS_VCORERDY                     (1 << 10)
    uint32_t bod33;
#define SUPC_BOD33_ENABLE                        (1 << 1)
#define SUPC_BOD33_HYST                          (1 << 2)
#define SUPC_BOD33_ACTION(val)                   (((val) & 0x3) << 3)
#define SUPC_BOD33_STDBYCFG                      (1 << 5)
#define SUPC_BOD33_RUNSTDBY                      (1 << 6)
#define SUPC_BOD33_RUNBKUP                       (1 << 7)
#define SUPC_BOD33_ACTCFG                        (1 << 8)
#define SUPC_BOD33_VMON                          (1 << 10)
#define SUPC_BOD33_PSEL(val)                     (((val) & 0xf) << 12)
#define SUPC_BOD33_LEVEL(val)                    (((val) & 0x3f) << 16)
#define SUPC_BOD33_BKUPLEVEL(val)                (((val) & 0x3f) << 24)
    uint32_t bod12;
#define SUPC_BOD12_ENABLE                        (1 << 1)
#define SUPC_BOD12_HYST                          (1 << 2)
#define SUPC_BOD12_ACTION(val)                   (((val) & 0x3) << 3)
#define SUPC_BOD12_STDBYCFG                      (1 << 5)
#define SUPC_BOD12_RUNSTDBY                      (1 << 6)
#define SUPC_BOD12_ACTCFG                        (1 << 8)
#define SUPC_BOD12_PSEL(val)                     (((val) & 0xf) << 12)
#define SUPC_BOD12_LEVEL(val)                    (((val) & 0x3f) << 16)
    uint32_t vreg;
#define SUPC_VREG_ENABLE                         (1 << 1)
#define SUPC_VREG_SEL                            (1 << 2)
#define SUPC_VREG_STDBYPL0                       (1 << 5)
#define SUPC_VREG_RUNSTDBY                       (1 << 6)
#define SUPC_VREG_LPEFF                          (1 << 8)
#define SUPC_VREG_VSVSTEP(val)                   (((val) & 0xf) << 16)
#define SUPC_VREG_VSPER(val)                     (((val) & 0xff) << 24)
    uint32_t vref;
#define SUPC_VREF_TSEN                           (1 << 1)
#define SUPC_VREF_VREFOE                         (1 << 2)
#define SUPC_VREF_TSSEL                          (1 << 3)
#define SUPC_VREF_RUNSTDBY                       (1 << 6)
#define SUPC_VREF_ONDEMAND                       (1 << 7)
#define SUPC_VREF_SEL(val)                       (((val) & 0xf) << 16)
#define SUPC_VREF_SEL_1V0                        (0x0 << 16)
#define SUPC_VREF_SEL_1V1                        (0x1 << 16)
#define SUPC_VREF_SEL_1V2                        (0x2 << 16)
#define SUPC_VREF_SEL_1V25                       (0x3 << 16)
#define SUPC_VREF_SEL_2V0                        (0x4 << 16)
#define SUPC_VREF_SEL_2V2                        (0x5 << 16)
#define SUPC_VREF_SEL_2V4                        (0x6 << 16)
#define SUPC_VREF_SEL_2V5                        (0x7 << 16)
    uint32_t bbps;
#define SUPC_BBPS_CONF(val)                      (((val) & 0x3) << 0)
#define SUPC_BBPS_WAKEEN                         (1 << 2)
#define SUPC_BBPS_PSOKEN                         (1 << 3)
    uint32_t bkout;
#define SUPC_BKOUT_EN(val)                       (((val) & 0x3) << 0)
#define SUPC_BKOUT_CLR(val)                      (((val) & 0x3) << 8)
#define SUPC_BKOUT_SET(val)                      (((val) & 0x3) << 16)
#define SUPC_BKOUT_RTCTGL(val)                   (((val) & 0x3) << 24)
    uint32_t bkin;
#define SUPC_BKIN_BKIN(val)                      (((val) & 0x7) << 0)
} __attribute__ ((packed)) supc_t;

#if defined(__AT91SAML21__)
#define SUPC_BASE                                0x40001400
#endif

#if defined(__ATSAMD53__)
#define SUPC_BASE                                0x40001800
#endif

#define SUPC                                     ((volatile supc_t *)SUPC_BASE)

#endif /* __SAML_POWER_H__ */
