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


#ifndef __SAML_CLOCKS_H__
#define __SAML_CLOCKS_H__

#ifdef __AT91SAML21__

//
// Peripheral Channels
//
#define GCLK_DFLL48M_REF                         0
#define GCLK_DPLL                                1
#define GCLK_DPLL_32K                            2
#define GCLK_EIC                                 3
#define GCLK_USB                                 4
#define GCLK_EVSYS_CHANNEL_0                     5
#define GCLK_EVSYS_CHANNEL_1                     6
#define GCLK_EVSYS_CHANNEL_2                     7
#define GCLK_EVSYS_CHANNEL_3                     8
#define GCLK_EVSYS_CHANNEL_4                     9
#define GCLK_EVSYS_CHANNEL_5                     10
#define GCLK_EVSYS_CHANNEL_6                     11
#define GCLK_EVSYS_CHANNEL_7                     12
#define GCLK_EVSYS_CHANNEL_8                     13
#define GCLK_EVSYS_CHANNEL_9                     14
#define GCLK_EVSYS_CHANNEL_10                    15
#define GCLK_EVSYS_CHANNEL_11                    16
#define GCLK_SERCOM0_SLOW                        17
#define GCLK_SERCOM1_SLOW                        17
#define GCLK_SERCOM2_SLOW                        17
#define GCLK_SERCOM3_SLOW                        17
#define GCLK_SERCOM4_SLOW                        17
#define GCLK_SERCOM0_CORE                        18
#define GCLK_SERCOM1_CORE                        19
#define GCLK_SERCOM2_CORE                        20
#define GCLK_SERCOM3_CORE                        21
#define GCLK_SERCOM4_CORE                        22
#define GCLK_SERCOM5_SLOW                        23
#define GCLK_SERCOM5_CORE                        24
#define GCLK_TCC0                                25
#define GCLK_TCC1                                25
#define GCLK_TCC2                                26
#define GCLK_TC0                                 27
#define GCLK_TC1                                 27
#define GCLK_TC2                                 28
#define GCLK_TC3                                 28
#define GCLK_TC4                                 29
#define GCLK_ADC                                 30
#define GCLK_AC                                  31
#define GCLK_DAC                                 32
#define GCLK_PTC                                 33
#define GCLK_CCL                                 34


typedef struct gclk
{
    uint32_t ctrla;
    uint32_t syncbusy;
#define GCLK_SYNCBUSY_SWRST                      (1 << 0)
#define GCLK_SYNCBUSY_GENCTRL0                   (1 << 2)
#define GCLK_SYNCBUSY_GENCTRL1                   (1 << 3)
#define GCLK_SYNCBUSY_GENCTRL2                   (1 << 4)
#define GCLK_SYNCBUSY_GENCTRL3                   (1 << 5)
#define GCLK_SYNCBUSY_GENCTRL4                   (1 << 6)
#define GCLK_SYNCBUSY_GENCTRL5                   (1 << 7)
#define GCLK_SYNCBUSY_GENCTRL6                   (1 << 13)
#define GCLK_SYNCBUSY_GENCTRL7                   (1 << 14)
#define GCLK_SYNCBUSY_GENCTRL8                   (1 << 15)
    uint8_t  resvd_0x08[24];
    uint32_t genctrl[9];
#define GCLK_GENCTRL_SRC_XOSC                    (0x0 << 0)
#define GCLK_GENCTRL_SRC_GCLK_IN                 (0x1 << 0)
#define GCLK_GENCTRL_SRC_GCLK_GEN1               (0x2 << 0)
#define GCLK_GENCTRL_SRC_OSCULP32K               (0x3 << 0)
#define GCLK_GENCTRL_SRC_OSC32K                  (0x4 << 0)
#define GCLK_GENCTRL_SRC_XOSC32K                 (0x5 << 0)
#define GCLK_GENCTRL_SRC_OSC16M                  (0x6 << 0)
#define GCLK_GENCTRL_SRC_DFLL48M                 (0x7 << 0)
#define GCLK_GENCTRL_SRC_DPLL96M                 (0x8 << 0)
#define GCLK_GENCTRL_GENEN                       (1 << 8)
#define GCLK_GENCTRL_IDC                         (1 << 9)
#define GCLK_GENCTRL_OOV                         (1 << 10)
#define GCLK_GENCTRL_OE                          (1 << 11)
#define GCLK_GENCTRL_DIVSEL                      (1 << 12)
#define GCLK_GENCTRL_RUNSTDBY                    (1 << 13)
#define GCLK_GENCTRL_DIV(val)                    (((val) & 0xffff) << 16)
    uint8_t  resvd_0x44[60];
    uint32_t pchctrl[35];
#define GCLK_PCHCTRL_GEN(val)                    (((val) & 0xf) << 0)
#define GCLK_PCHCTRL_CHEN                        (1 << 6)
#define GCLK_PCHCTRL_WRTLOCK                     (1 << 7)
} __attribute__((packed)) gclk_t;

#define GCLK_BASE                                0x40001800
#define GCLK                                     ((volatile gclk_t *)GCLK_BASE)


typedef struct mclk
{
    uint8_t ctrla;
    uint8_t intenclr;
#define MCLK_INTENCLR_CKRDY                      (1 << 0)
    uint8_t intenset;
#define MCLK_INTENSET_CKRDY                      (1 << 0)
    uint8_t intflag;
#define MCLK_INTENFLAG_CKRDY                     (1 << 0)
    uint8_t resvd_0x04;
    uint8_t cpudiv;
    uint8_t resvd_0x06[10];
    uint32_t ahbmask;
#define MCLK_AHBMASK_APBA                        (1 << 0)
#define MCLK_AHBMASK_APBB                        (1 << 1)
#define MCLK_AHBMASK_APBC                        (1 << 2)
#define MCLK_AHBMASK_APBD                        (1 << 3)
#define MCLK_AHBMASK_APBE                        (1 << 4)
#define MCLK_AHBMASK_DSU                         (1 << 5)
#define MCLK_AHBMASK_NVMCTRL                     (1 << 8)
#define MCLK_AHBMASK_DMAC                        (1 << 11)
#define MCLK_AHBMASK_USB                         (1 << 12)
#define MCLK_AHBMASK_PAC                         (1 << 14)
    uint32_t apbamask;
#define MCLK_APBAMASK_PM                         (1 << 0)
#define MCLK_APBAMASK_MCLK                       (1 << 1)
#define MCLK_APBAMASK_RSTC                       (1 << 2)
#define MCLK_APBAMASK_OSCCTRL                    (1 << 3)
#define MCLK_APBAMASK_OSC32KCTRL                 (1 << 4)
#define MCLK_APBAMASK_SUPC                       (1 << 5)
#define MCLK_APBAMASK_GCLK                       (1 << 6)
#define MCLK_APBAMASK_WDT                        (1 << 7)
#define MCLK_APBAMASK_RTC                        (1 << 8)
#define MCLK_APBAMASK_EIC                        (1 << 9)
#define MCLK_APBAMASK_PORT                       (1 << 10)
    uint32_t abpbmask;
#define MCLK_APBBMASK_USB                        (1 << 0)
#define MCLK_APBBMASK_DSU                        (1 << 1)
#define MCLK_APBBMASK_NVMCTRL                    (1 << 2)
    uint32_t apbcmask;
#define MCLK_APBCMASK_SERCOM0                    (1 << 0)
#define MCLK_APBCMASK_SERCOM1                    (1 << 1)
#define MCLK_APBCMASK_SERCOM2                    (1 << 2)
#define MCLK_APBCMASK_SERCOM3                    (1 << 3)
#define MCLK_APBCMASK_SERCOM4                    (1 << 4)
#define MCLK_APBCMASK_TCC0                       (1 << 5)
#define MCLK_APBCMASK_TCC1                       (1 << 6)
#define MCLK_APBCMASK_TCC2                       (1 << 7)
#define MCLK_APBCMASK_TC0                        (1 << 8)
#define MCLK_APBCMASK_TC1                        (1 << 9)
#define MCLK_APBCMASK_TC2                        (1 << 10)
#define MCLK_APBCMASK_TC3                        (1 << 11)
#define MCLK_APBCMASK_DAC                        (1 << 12)
#define MCLK_APBCMASK_AES                        (1 << 13)
#define MCLK_APBCMASK_TRNG                       (1 << 14)
    uint32_t apbdmask;
#define MCLK_APBDMASK_EVSYS                      (1 << 0)
#define MCLK_APBDMASK_SERCOM5                    (1 << 1)
#define MCLK_APBDMASK_TC4                        (1 << 2)
#define MCLK_APBDMASK_ADC                        (1 << 3)
#define MCLK_APBDMASK_AC                         (1 << 4)
#define MCLK_APBDMASK_PTC                        (1 << 5)
#define MCLK_APBDMASK_OPAMP                      (1 << 6)
#define MCLK_APBDMASK_CCL                        (1 << 7)
    uint32_t apbemask;
#define MCLK_APBEMASK_PAC                        (1 << 0)
} __attribute__((packed)) mclk_t;


#define MCLK_BASE                                0x40000400
#define MCLK                                     ((volatile mclk_t *)MCLK_BASE)


typedef struct oscctrl
{
    uint32_t intenclr;
#define OSCCTRL_INTENCLR_XOSCRDY                 (1 << 0)
#define OSCCTRL_INTENCLR_OSC16MRDY               (1 << 4)
#define OSCCTRL_INTENCLR_DFLLRDY                 (1 << 8)
#define OSCCTRL_INTENCLR_DFLLOOB                 (1 << 9)
#define OSCCTRL_INTENCLR_DFLLLCKF                (1 << 10)
#define OSCCTRL_INTENCLR_DFLLLCKC                (1 << 11)
#define OSCCTRL_INTENCLR_DFLLRCS                 (1 << 12)
#define OSCCTRL_INTENCLR_DPLLLCKR                (1 << 16)
#define OSCCTRL_INTENCLR_CPLLLCKF                (1 << 17)
#define OSCCTRL_INTENCLR_CPLLLTO                 (1 << 18)
#define OSCCTRL_INTENCLR_DPLLLDRTO               (1 << 19)
    uint32_t intenset;
#define OSCCTRL_INTENSET_XOSCRDY                 (1 << 0)
#define OSCCTRL_INTENSET_OSC16MRDY               (1 << 4)
#define OSCCTRL_INTENSET_DFLLRDY                 (1 << 8)
#define OSCCTRL_INTENSET_DFLLOOB                 (1 << 9)
#define OSCCTRL_INTENSET_DFLLLCKF                (1 << 10)
#define OSCCTRL_INTENSET_DFLLLCKC                (1 << 11)
#define OSCCTRL_INTENSET_DFLLRCS                 (1 << 12)
#define OSCCTRL_INTENSET_DPLLLCKR                (1 << 16)
#define OSCCTRL_INTENSET_CPLLLCKF                (1 << 17)
#define OSCCTRL_INTENSET_CPLLLTO                 (1 << 18)
#define OSCCTRL_INTENSET_DPLLLDRTO               (1 << 19)
    uint32_t intflag;
#define OSCCTRL_INTFLAG_XOSCRDY                  (1 << 0)
#define OSCCTRL_INTFLAG_OSC16MRDY                (1 << 4)
#define OSCCTRL_INTFLAG_DFLLRDY                  (1 << 8)
#define OSCCTRL_INTFLAG_DFLLOOB                  (1 << 9)
#define OSCCTRL_INTFLAG_DFLLLCKF                 (1 << 10)
#define OSCCTRL_INTFLAG_DFLLLCKC                 (1 << 11)
#define OSCCTRL_INTFLAG_DFLLRCS                  (1 << 12)
#define OSCCTRL_INTFLAG_DPLLLCKR                 (1 << 16)
#define OSCCTRL_INTFLAG_CPLLLCKF                 (1 << 17)
#define OSCCTRL_INTFLAG_CPLLLTO                  (1 << 18)
#define OSCCTRL_INTFLAG_DPLLLDRTO                (1 << 19)
    uint32_t status;
#define OSCCTRL_STATUS_XOSCRDY                   (1 << 0)
#define OSCCTRL_STATUS_OSC16MRDY                 (1 << 4)
#define OSCCTRL_STATUS_DFLLRDY                   (1 << 8)
#define OSCCTRL_STATUS_DFLLOOB                   (1 << 9)
#define OSCCTRL_STATUS_DFLLLCKF                  (1 << 10)
#define OSCCTRL_STATUS_DFLLLCKC                  (1 << 11)
#define OSCCTRL_STATUS_DFLLRCS                   (1 << 12)
#define OSCCTRL_STATUS_DPLLLCKR                  (1 << 16)
#define OSCCTRL_STATUS_CPLLLCKF                  (1 << 17)
#define OSCCTRL_STATUS_CPLLLTO                   (1 << 18)
#define OSCCTRL_STATUS_DPLLLDRTO                 (1 << 19)
    uint16_t xoscctrl;
#define OSCCTRL_XOSCCTRL_ENABLE                  (1 << 1)
#define OSCCTRL_XOSCCTRL_XTALEN                  (1 << 2)
#define OSCCTRL_XOSCCTRL_RUNSTDBY                (1 << 6)
#define OSCCTRL_XOSCCTRL_ONDEMAND                (1 << 7)
#define OSCCTRL_XOSCCTRL_GAIN(val)               (((val) & 0x7) << 8)
#define OSCCTRL_XOSCCTRL_AMPGC                   (1 << 11)
#define OSCCTRL_XOSCCTRL_STARTUP(val)            (((val) & 0xf) << 12)
    uint16_t resvd_0x12;
    uint8_t  osc16mctrl;
#define OSCCTRL_OSC16MCTRL_ENABLE                (1 << 1)
#define OSCCTRL_OSC16MCTRL_FSEL_MASK             (0x3 << 2)
#define OSCCTRL_OSC16MCTRL_FSEL_4Mhz             (0x0 << 2)
#define OSCCTRL_OSC16MCTRL_FSEL_8Mhz             (0x1 << 2)
#define OSCCTRL_OSC16MCTRL_FSEL_12Mhz            (0x2 << 2)
#define OSCCTRL_OSC16MCTRL_FSEL_16Mhz            (0x3 << 2)
#define OSCCTRL_OSC16MCTRL_RUNSTDBY              (1 << 6)
#define OSCCTRL_OSC16MCTRL_ONDEMAND              (1 << 7)
    uint8_t  resvd_0x15[3];
    uint16_t dfllctrl;
#define OSCCTRL_DFLLCTRL_ENABLE                  (1 << 1)
#define OSCCTRL_DFLLCTRL_MODE                    (1 << 2)
#define OSCCTRL_DFLLCTRL_STABLE                  (1 << 3)
#define OSCCTRL_DFLLCTRL_LLAW                    (1 << 4)
#define OSCCTRL_DFLLCTRL_USBCRM                  (1 << 5)
#define OSCCTRL_DFLLCTRL_RUNSTDBY                (1 << 6)
#define OSCCTRL_DFLLCTRL_ONDEMAND                (1 << 7)
#define OSCCTRL_DFLLCTRL_CCDIS                   (1 << 8)
#define OSCCTRL_DFLLCTRL_QLDIS                   (1 << 9)
#define OSCCTRL_DFLLCTRL_BPLCKC                  (1 << 10)
#define OSCCTRL_DFLLCTRL_WAITLOCK                (1 << 11)
    uint16_t resvd_0x1a;
    uint32_t dfllval;
#define OSCCTRL_DFLLVAL_FINE(val)                (((val) & 0x3ff) << 0)
#define OSCCTRL_DFLLVAL_COARSE(val)              (((val) & 0x3f) << 10)
#define OSCCTRL_DFLLVAL_DIFF(val)                (((val) & 0xffff) << 16)
    uint32_t dfllmul;
#define OSCCTRL_DFLLMUL_MUL(val)                 (((val) & 0xffff) << 0)
#define OSCCTRL_DFLLMUL_FSTEP(val)               (((val) & 0x3ff) << 16)
#define OSCCTRL_DFLLMUL_CSTEP(val)               (((val) & 0x3f) << 26)
    uint8_t  dfllsync;
#define OSCCTRL_DFLLSYNC_READREQ                 (1 << 7)
    uint8_t  resvd_0x25[3];
    uint8_t  dpllctrla;
#define OSCCTRL_DPLLCTRLA_ENABLE                 (1 << 1)
#define OSCCTRL_DPLLCTRLA_RUNSTDBY               (1 << 6)
#define OSCCTRL_DPLLCTRLA_ONDEMAND               (1 << 7)
    uint8_t  resvd_0x29[3];
    uint32_t dpllratio;
#define OSCCTRL_DPLLRATIO_LDR(val)               (((val) & 0xfff) << 0)
#define OSCCTRL_DPLLRATIO_LDRFRAC(val)           (((val) & 0xf) << 16)
    uint32_t dpllctrlb;
#define OSCCTRL_DPLLCTRLB_FILTER(val)            (((val) & 0x3) << 0)
#define OSCCTRL_DPLLCTRLB_LPEN                   (1 << 2)
#define OSCCTRL_DPLLCTRLB_WUF                    (1 << 3)
#define OSCCTRL_DPLLCTRLB_REFCLK_XOSC32K         (0x0 << 4)
#define OSCCTRL_DPLLCTRLB_REFCLK_XOSC            (0x1 << 4)
#define OSCCTRL_DPLLCTRLB_REFCLK_GCLK            (0x2 << 4)
#define OSCCTRL_DPLLCTRLB_LTIME(val)             (((val) & 0x7) << 8)
#define OSCCTRL_DPLLCTRLB_LBYPASS                (1 << 12)
#define OSCCTRL_DPLLCTRLB_DIV(val)               (((val) & 0x3ff) << 16)
    uint8_t dpllpresc;
#define OSCCTRL_DPLLCTRLB_DPLLPRESC(val)         (((val) & 0x3) << 0)
    uint8_t  resvd_0x35[3];
    uint8_t  dpllsyncbusy;
#define OSCCTRL_DPLLSYNCBUSY_ENABLE              (1 << 1)
#define OSCCTRL_DPLLSYNCBUSY_DPLLRATIO           (1 << 2)
#define OSCCTRL_DPLLSYNCBUSY_DPLLPRESC           (1 << 3)
    uint8_t  resvd_0x39[3];
    uint8_t  dpllstatus;
#define OSCCTRL_DPLLSTATUS_LOCK                  (1 << 0)
#define OSCCTRL_DPLLSTATUS_CLKRDY                (1 << 1)
} __attribute__((packed)) oscctrl_t;


#define OSCCTRL_BASE                             0x40000c00
#define OSCCTRL                                  ((volatile oscctrl_t *)OSCCTRL_BASE)

#define DFLLCTRL_COARSE_VAL                      (*((uint32_t *)0x00806020) >> 26)

typedef struct osc32kctrl
{
    uint32_t intenclr;
#define OSC32KCTRL_INTENCLR_XOSC32KRDY           (1 << 0)
#define OSC32KCTRL_INTENCLR_OSC32KRDY            (1 << 1)
    uint32_t intenset;
#define OSC32KCTRL_INTENSET_XOSC32KRDY           (1 << 0)
#define OSC32KCTRL_INTENSET_OSC32KRDY            (1 << 1)
    uint32_t intflag;
#define OSC32KCTRL_INTFLAG_XOSC32KRDY            (1 << 0)
#define OSC32KCTRL_INTFLAG_OSC32KRDY             (1 << 1)
    uint32_t status;
#define OSC32KCTRL_STATUS_XOSC32KRDY             (1 << 0)
#define OSC32KCTRL_STATUS_OSC32KRDY              (1 << 1)
    uint32_t rtcctrl;
#define OSC32KCTRL_RTCCTRL_RTCSEL(val)           (((val) & 0x7) << 0)
    uint32_t xosc32k;
#define OSC32KCTRL_XOSC32K_ENABLE                (1 << 1)
#define OSC32KCTRL_XOSC32K_XTALEN                (1 << 2)
#define OSC32KCTRL_XOSC32K_EN32K                 (1 << 3)
#define OSC32KCTRL_XOSC32K_EN1K                  (1 << 4)
#define OSC32KCTRL_XOSC32K_RUNSTDBY              (1 << 6)
#define OSC32KCTRL_XOSC32K_ONDEMAND              (1 << 7)
#define OSC32KCTRL_XOSC32K_STARTUP(val)          (((val) & 0x7) << 8)
#define OSC32KCTRL_XOSC32K_WRTLOCK               (1 << 12)
    uint32_t osc32k;
#define OSC32KCTRL_OSC32K_ENABLE                 (1 << 1)
#define OSC32KCTRL_OSC32K_EN32K                  (1 << 2)
#define OSC32KCTRL_OSC32K_EN1K                   (1 << 3)
#define OSC32KCTRL_OSC32K_RUNSTDBY               (1 << 6)
#define OSC32KCTRL_OSC32K_ONDEMAND               (1 << 7)
#define OSC32KCTRL_OSC32K_STARTUP(val)           (((val) & 0x7) << 8)
#define OSC32KCTRL_OSC32K_WRTLOCK                (1 << 12)
#define OSC32KCTRL_OSC32K_CALIB(val)             (((val) & 0x7f) << 16)
    uint16_t osculp32k;
#define OSC32KCTRL_OSCULP32K_CALIB(val)          (((val) & 0x1f) << 8)
#define OSC32KCTRL_OSCULP32K_WRTLOCK             (1 << 15)
} __attribute__((packed)) osc32kctrl_t;


#define OSC32KCTRL_BASE                          0x40001000
#define OSC32KCTRL                               ((volatile osc32kctrl_t *)OSC32KCTRL_BASE)

#define OSC32KCAL                                ((*((uint32_t *)0x00806020) >> 6) & 0x3f)

#endif /* __AT91SAML21__ */

#ifdef __AT91SAMD20__

#define GCLK_DFLL48M_REF                         0
#define GCLK_WDT                                 1
#define GCLK_RTC                                 2
#define GCLK_EIC                                 3
#define GCLK_EVSYS_CHANNEL_0                     4
#define GCLK_EVSYS_CHANNEL_1                     5
#define GCLK_EVSYS_CHANNEL_2                     6
#define GCLK_EVSYS_CHANNEL_3                     7
#define GCLK_EVSYS_CHANNEL_4                     8
#define GCLK_EVSYS_CHANNEL_5                     9
#define GCLK_EVSYS_CHANNEL_6                     10
#define GCLK_EVSYS_CHANNEL_7                     11
#define GCLK_SERCOMx_SLOW                        12
#define GCLK_SERCOM0_CORE                        13
#define GCLK_SERCOM1_CORE                        14
#define GCLK_SERCOM2_CORE                        15
#define GCLK_SERCOM3_CORE                        16
#define GCLK_SERCOM4_CORE                        17
#define GCLK_SERCOM5_CORE                        18
#define GCLK_TC0                                 19
#define GCLK_TC1                                 19
#define GCLK_TC2                                 20
#define GCLK_TC3                                 20
#define GCLK_TC4                                 21
#define GCLK_TC5                                 21
#define GCLK_TC6                                 22
#define GCLK_TC7                                 22
#define GCLK_ADC                                 23
#define GCLK_AC_DIGITAL                          24
#define GCLK_AC_ANALOG                           25
#define GCLK_DAC                                 26
#define GCLK_PTC                                 27


typedef struct gclk
{
    uint8_t ctrl;
#define GCLK_CTRL_SWRST                          (1 << 0)
    uint8_t status;
#define GCLK_STATUS_SYNCBUSY                     (1 << 7)
    uint16_t clkctrl;
#define GCLK_CLKCTRL_ID(val)                     ((val & 0x7f) << 0)
#define GCLK_CLKCTRL_GEN(val)                    ((val & 0xf) << 8)
#define GCLK_CLKCTRL_CLKEN                       (1 << 14)
#define GCLK_CLKCTRL_WRTLOCK                     (1 << 15)
    uint32_t genctrl;
#define GCLK_GENCTRL_ID(val)                     ((val & 0xf) << 0)
#define GCLK_GENCTRL_SRC(val)                    ((val & 0x1f) << 8)
#define GCLK_GENCTRL_SRC_XOSC                    0
#define GCLK_GENCTRL_SRC_GCLKIN                  1
#define GCLK_GENCTRL_SRC_GCLKGEN1                2
#define GCLK_GENCTRL_SRC_OSCULP32K               3
#define GCLK_GENCTRL_SRC_OSC32K                  4
#define GCLK_GENCTRL_SRC_XOSC32K                 5
#define GCLK_GENCTRL_SRC_OSC8M                   6
#define GCLK_GENCTRL_SRC_DFLL48M                 7
#define GCLK_GENCTRL_GENEN                       (1 << 16)
#define GCLK_GENCTRL_IDC                         (1 << 17)
#define GCLK_GENCTRL_OOV                         (1 << 18)
#define GCLK_GENCTRL_OE                          (1 << 19)
#define GCLK_GENCTRL_DIVSEL                      (1 << 20)
#define GCLK_GENCTRL_RUNSTDBY                    (1 << 21)
    uint32_t gendiv;
#define GCLK_GENDIV_ID(val)                      ((val & 0xf) << 0)
#define GCLK_GENDIV_DIV(val)                     ((val & 0xffff) << 8)
} __attribute__((packed)) gclk_t;

#define GCLK_BASE                                0x40000c00
#define GCLK                                     ((volatile gclk_t *)GCLK_BASE)


typedef struct pm
{
    uint8_t ctrl;
    uint8_t sleep;
#define PM_SLEEP_IDLE(val)                       ((val & 0x3) << 0)
    uint8_t resvd_0x02[6];
    uint8_t cpusel;
#define PM_CPUSEL_CPUDIV(val)                    ((val & 0x7) << 0)
    uint8_t apbasel;
#define PM_APBASEL_APBADIV(val)                  ((val & 0x7) << 0)
    uint8_t apbbsel;
#define PM_APBBSEL_APBBDIV(val)                  ((val & 0x7) << 0)
    uint8_t apbcsel;
#define PM_APBCSEL_APBCDIV(val)                  ((val & 0x7) << 0)
    uint8_t resvd_0x08[8];
    uint32_t ahbmask;
    uint32_t apbamask;
    uint32_t apbbmask;
    uint32_t apbcmask;
#define PM_APBCMASK_PAC2                         (1 << 0)
#define PM_APBCMASK_EVSYS                        (1 << 1)
#define PM_APBCMASK_SERCOM0                      (1 << 2)
#define PM_APBCMASK_SERCOM1                      (1 << 3)
#define PM_APBCMASK_SERCOM2                      (1 << 4)
#define PM_APBCMASK_SERCOM3                      (1 << 5)
#define PM_APBCMASK_SERCOM4                      (1 << 6)
#define PM_APBCMASK_SERCOM5                      (1 << 7)
#define PM_APBCMASK_TC0                          (1 << 8)
#define PM_APBCMASK_TC1                          (1 << 9)
#define PM_APBCMASK_TC2                          (1 << 10)
#define PM_APBCMASK_TC3                          (1 << 11)
#define PM_APBCMASK_TC4                          (1 << 12)
#define PM_APBCMASK_TC5                          (1 << 13)
#define PM_APBCMASK_TC6                          (1 << 14)
#define PM_APBCMASK_TC7                          (1 << 15)
#define PM_APBCMASK_ADC                          (1 << 16)
#define PM_APBCMASK_AC                           (1 << 17)
#define PM_APBCMASK_DAC                          (1 << 18)
#define PM_APBCMASK_PTC                          (1 << 19)
    uint8_t resvd_0x24[16];
    uint8_t intenclr;
#define PM_INTENCLR_CKRDY                        (1 << 0)
    uint8_t intenset;
#define PM_INTENSET_CKRDY                        (1 << 0)
    uint8_t intflag;
#define PM_INTFLAG_CKRDY                         (1 << 0)
    uint8_t resvd_0x37;
    uint8_t rcause;
#define PM_RCAUSE_POR                            (1 << 0)
#define PM_RCAUSE_BOD12                          (1 << 1)
#define PM_RCAUSE_BOD33                          (1 << 2)
#define PM_RCAUSE_EXT                            (1 << 4)
#define PM_RCAUSE_WDT                            (1 << 5)
#define PM_RCAUSE_SYST                           (1 << 6)
} __attribute__((packed)) pm_t;

#define PM_BASE                                  0x40000400
#define PM                                       ((volatile pm_t *)PM_BASE)


typedef struct sysctrl
{
    uint32_t intenclr;
#define SYSCTRL_INTENCLR_XOSCRDY                 (1 << 0)
#define SYSCTRL_INTENCLR_XOSC32KRDY              (1 << 1)
#define SYSCTRL_INTENCLR_OSC32KRDY               (1 << 2)
#define SYSCTRL_INTENCLR_OSC8MRY                 (1 << 3)
#define SYSCTRL_INTENCLR_DFLLRDY                 (1 << 4)
#define SYSCTRL_INTENCLR_DFLLOOB                 (1 << 5)
#define SYSCTRL_INTENCLR_DFLLLCKF                (1 << 6)
#define SYSCTRL_INTENCLR_DFLLLCKC                (1 << 7)
#define SYSCTRL_INTENCLR_DFLLRCS                 (1 << 8)
#define SYSCTRL_INTENCLR_BOD33RDY                (1 << 9)
#define SYSCTRL_INTENCLR_DOB33DET                (1 << 10)
#define SYSCTRL_INTENCLR_B33SRDY                 (1 << 11)
    uint32_t intenset;
#define SYSCTRL_INTENSET_XOSCRDY                 (1 << 0)
#define SYSCTRL_INTENSET_XOSC32KRDY              (1 << 1)
#define SYSCTRL_INTENSET_OSC32KRDY               (1 << 2)
#define SYSCTRL_INTENSET_OSC8MRY                 (1 << 3)
#define SYSCTRL_INTENSET_DFLLRDY                 (1 << 4)
#define SYSCTRL_INTENSET_DFLLOOB                 (1 << 5)
#define SYSCTRL_INTENSET_DFLLLCKF                (1 << 6)
#define SYSCTRL_INTENSET_DFLLLCKC                (1 << 7)
#define SYSCTRL_INTENSET_DFLLRCS                 (1 << 8)
#define SYSCTRL_INTENSET_BOD33RDY                (1 << 9)
#define SYSCTRL_INTENSET_DOB33DET                (1 << 10)
#define SYSCTRL_INTENSET_B33SRDY                 (1 << 11)
    uint32_t intflag;
#define SYSCTRL_INTENFLAG_XOSCRDY                (1 << 0)
#define SYSCTRL_INTENFLAG_XOSC32KRDY             (1 << 1)
#define SYSCTRL_INTENFLAG_OSC32KRDY              (1 << 2)
#define SYSCTRL_INTENFLAG_OSC8MRY                (1 << 3)
#define SYSCTRL_INTENFLAG_DFLLRDY                (1 << 4)
#define SYSCTRL_INTENFLAG_DFLLOOB                (1 << 5)
#define SYSCTRL_INTENFLAG_DFLLLCKF               (1 << 6)
#define SYSCTRL_INTENFLAG_DFLLLCKC               (1 << 7)
#define SYSCTRL_INTENFLAG_DFLLRCS                (1 << 8)
#define SYSCTRL_INTENFLAG_BOD33RDY               (1 << 9)
#define SYSCTRL_INTENFLAG_DOB33DET               (1 << 10)
#define SYSCTRL_INTENFLAG_B33SRDY                (1 << 11)
    uint32_t pclksr;
#define SYSCTRL_PCLKSR_XOSCRDY                   (1 << 0)
#define SYSCTRL_PCLKSR_XOSC32KRDY                (1 << 1)
#define SYSCTRL_PCLKSR_OSC32KRDY                 (1 << 2)
#define SYSCTRL_PCLKSR_OSC8MRY                   (1 << 3)
#define SYSCTRL_PCLKSR_DFLLRDY                   (1 << 4)
#define SYSCTRL_PCLKSR_DFLLOOB                   (1 << 5)
#define SYSCTRL_PCLKSR_DFLLLCKF                  (1 << 6)
#define SYSCTRL_PCLKSR_DFLLLCKC                  (1 << 7)
#define SYSCTRL_PCLKSR_DFLLRCS                   (1 << 8)
#define SYSCTRL_PCLKSR_BOD33RDY                  (1 << 9)
#define SYSCTRL_PCLKSR_DOB33DET                  (1 << 10)
#define SYSCTRL_PCLKSR_B33SRDY                   (1 << 11)
    uint16_t xosc;
#define SYSCTRL_OSC_ENABLE                       (1 << 0)
#define SYSCTRL_OSC_XTALEN                       (1 << 1)
#define SYSCTRL_OSC_RUNSTDBY                     (1 << 2)
#define SYSCTRL_OSC_ONDEMAND                     (1 << 3)
#define SYSCTRL_OSC_GAIN(val)                    ((val & 0x7) << 8)
#define SYSCTRL_OSC_AMPGC                        (1 << 11)
#define SYSCTRL_OSC_STARTUP(val)                 ((val & 0xf) << 12)
    uint16_t resvd_0x12;
    uint16_t xosc32k;
#define SYSCTRL_XOSC32K_ENABLE                   (1 << 1)
#define SYSCTRL_XOSC32K_XTALEN                   (1 << 2)
#define SYSCTRL_XOSC32K_EN32K                    (1 << 3)
#define SYSCTRL_XOSC32K_AAMPEN                   (1 << 5)
#define SYSCTRL_XOSC32K_RUNSTDBY                 (1 << 6)
#define SYSCTRL_XOSC32K_ONDEMAND                 (1 << 7)
#define SYSCTRL_XOSC32K_STARTUP(val)             ((val & 0x7) << 8)
#define SYSCTRL_XOSC32K_WRTLOCK                  (1 << 12)
    uint16_t resvd_0x16;
    uint32_t osc32k;
#define SYSCTRL_OSC32K_ENABLE                    (1 << 1)
#define SYSCTRL_OSC32K_EN32K                     (1 << 2)
#define SYSCTRL_OSC32K_RUNSTDBY                  (1 << 6)
#define SYSCTRL_OSC32K_ONDEMAND                  (1 << 7)
#define SYSCTRL_OSC32K_STARTUP(val)              ((val & 0x7) << 8)
#define SYSCTRL_OSC32K_WRTLOCK                   (1 << 12)
#define SYSCTRL_OSC32K_CALIB(val)                ((val & 0x7f) << 16)
    uint8_t  osculp32k;
#define SYSCTRL_OSCULP32K_CALIB(val)             ((val & 1f) << 0)
#define SYSCTRL_OSCULP32K_WRTLOCK                (1 << 7)
    uint8_t  resvd_0x1d[3];
    uint32_t osc8m;
#define SYSCTRL_OSC8M_ENABLE                     (1 << 1)
#define SYSCTRL_OSC8M_RUNSTDBY                   (1 << 6)
#define SYSCTRL_OSC8M_ONDEMAND                   (1 << 7)
#define SYSCTRL_OSC8M_PRESC(val)                 ((val & 0x3) << 8)
#define SYSCTRL_OSC8M_CALIB(val)                 ((val & 0xfff) << 16)
#define SYSCTRL_OSC8M_FRANGE(val)                ((val & 0x3) << 30)
    uint16_t dfllctrl;
#define SYSCTRL_DFLLCTRL_ENABLE                  (1 << 1)
#define SYSCTRL_DFLLCTRL_MODE                    (1 << 2)
#define SYSCTRL_DFLLCTRL_STABLE                  (1 << 3)
#define SYSCTRL_DFLLCTRL_LLAW                    (1 << 4)
#define SYSCTRL_DFLLCTRL_ONDEMAND                (1 << 7)
#define SYSCTRL_DFLLCTRL_CCDIS                   (1 << 8)
#define SYSCTRL_DFLLCTRL_QLDIS                   (1 << 9)
    uint16_t resvd_0x26;
    uint32_t dfllval;
#define SYSCTRL_DFLLVAL_FINE(val)                ((val & 0x3ff) << 0)
#define SYSCTRL_DFLLVAL_COARSE(val)              ((val & 0x3f) << 10)
#define SYSCTRL_DFLLVAL_DIFF(val)                ((val & 0xffff) << 16)
    uint32_t dfllmul;
#define SYSCTRL_DFLLMUL_MUL(val)                 ((val & 0xffff) << 0)
#define SYSCTRL_DFLLMUL_FSTEP(val)               ((val & 0x3ff) << 16)
#define SYSCTRL_DFLLMUL_CSTEP(val)               ((val & 0x3f) << 26)
    uint8_t  dfllsync;
#define SYSCTRL_DFLLSYNC_READREQ                 (1 << 7)
    uint8_t  resvd_0x31[3];
    uint32_t bod33;
    uint32_t resvd_0x38;
    uint16_t vreg;
    uint16_t resvd_0x3e;
    uint32_t vref;
} __attribute__((packed)) sysctrl_t;

#define SYSCTRL_BASE                             0x40000800
#define SYSCTRL                                  ((volatile sysctrl_t *)SYSCTRL_BASE)


#endif /* __AT91SAMD20__ */

#ifdef __ATSAMD53__

#define GCLK_DFLL48M_REF                         0
#define GCLK_FDPLL0                              1
#define GCLK_FDPLL1                              2
#define GCLK_SLOW                                3
#define GCLK_EIC                                 4
#define GCLK_FREQM_MSR                           5
#define GCLK_FREQM_REF                           6
#define GCLK_SERCOM0_CORE                        7
#define GCLK_SERCOM1_CORE                        8
#define GCLK_TC0_1                               9
#define GCLK_USB                                 10
#define GCLK_EVSYS0                              11
#define GCLK_EVSYS1                              12
#define GCLK_EVSYS2                              13
#define GCLK_EVSYS3                              14
#define GCLK_EVSYS4                              15
#define GCLK_EVSYS5                              16
#define GCLK_EVSYS6                              17
#define GCLK_EVSYS7                              18
#define GCLK_EVSYS8                              19
#define GCLK_EVSYS9                              20 
#define GCLK_EVSYS10                             21
#define GCLK_EVSYS11                             22
#define GCLK_SERCOM2_CORE                        23
#define GCLK_SERCOM3_CORE                        24
#define GCLK_TCC0_1                              25
#define GCLK_TC2_3                               26
#define GCLK_CAN0                                27
#define GCLK_CAN1                                28
#define GCLK_TCC2_3                              29
#define GCLK_TC4_5                               30
#define GCLK_PDEC                                31
#define GCLK_AC                                  32
#define GCLK_CCL                                 33
#define GCLK_SERCOM4_CORE                        34
#define GCLK_SERCOM5_CORE                        35
#define GCLK_SERCOM6_CORE                        36
#define GCLK_SERCOM7_CORE                        37
#define GCLK_TCC4                                38
#define GCLK_TC6_7                               39
#define GCLK_ADC0                                40
#define GCLK_ADC1                                41
#define GCLK_DAC                                 42
#define GCLK_I2S0                                43
#define GCLK_I2S1                                44
#define GCLK_SDHC0                               45
#define GCLK_SDHC1                               46
#define GCLK_CM4_TRACE                           47


typedef struct gclk
{
    uint8_t  ctrla;
#define GCLK_CTRLA_SWRST                         (1 << 0)
    uint8_t  resvd_0x1[3];
    uint32_t syncbusy;
#define GCLK_SYNCBUSY_SWRST                      (1 << 0)
#define GCLK_SYNCBUSY_GENCTRL(val)               ((val & 0x1) << 2)
    uint32_t resvd_0x8[6];
    uint32_t genctrl[12];
#define GCLK_GENCTRL_SRC(val)                    ((val & 0x1f) << 0)
#define GCLK_GENCTRL_SRC_XOSC0                   ((0x0 & 0x1f) << 0)
#define GCLK_GENCTRL_SRC_XOSC1                   ((0x1 & 0x1f) << 0)
#define GCLK_GENCTRL_SRC_GCLK_IN                 ((0x2 & 0x1f) << 0)
#define GCLK_GENCTRL_SRC_GCLK_GEN1               ((0x3 & 0x1f) << 0)
#define GCLK_GENCTRL_SRC_OSCULP32K               ((0x4 & 0x1f) << 0)
#define GCLK_GENCTRL_SRC_XOSC32K                 ((0x5 & 0x1f) << 0)
#define GCLK_GENCTRL_SRC_DFLL                    ((0x6 & 0x1f) << 0)
#define GCLK_GENCTRL_SRC_DPLL0                   ((0x7 & 0x1f) << 0)
#define GCLK_GENCTRL_SRC_DPLL1                   ((0x8 & 0x1f) << 0)
#define GCLK_GENCTRL_GENEN                       (1 << 8)
#define GCLK_GENCTRL_IDC                         (1 << 9)
#define GCLK_GENCTRL_OOV                         (1 << 10)
#define GCLK_GENCTRL_OE                          (1 << 11)
#define GCLK_GENCTRL_DIVSEL                      (1 << 12)
#define GCLK_GENCTRL_RUNSTDBY                    (1 << 13)
#define GCLK_GENCTRL_DIV(val)                    ((val & 0xffff) << 16)
    uint32_t resvd_0x50[12];
    uint32_t pchctrl[48];
#define GCLK_PCHCTRL_GEN(val)                    ((val & 0xf) << 0)
#define GCLK_PCHCTRL_CHEN                        (1 << 6)
#define GCLK_PCHCTRL_WRTLOCK                     (1 << 7)
} __attribute__((packed)) gclk_t;

#define GCLK_BASE                                0x40001c00
#define GCLK                                     ((volatile gclk_t *)GCLK_BASE)


typedef struct mclk
{
    uint8_t ctrla;
    uint8_t intenclr;
#define MCLK_INTENCLR_CKRDY                      (1 << 0)
    uint8_t intenset;
#define MCLK_INTENSET_CKRDY                      (1 << 0)
    uint8_t intflag;
#define MCLK_INTENFLAG_CKRDY                     (1 << 0)
    uint8_t hsdiv;
    uint8_t cpudiv;
    uint8_t resvd_0x06[10];
    uint32_t ahbmask;
#define MCLK_AHBMASK_HPBn0                       (1 << 0)
#define MCLK_AHBMASK_HPBn1                       (1 << 1)
#define MCLK_AHBMASK_HPBn2                       (1 << 2)
#define MCLK_AHBMASK_HPBn3                       (1 << 3)
#define MCLK_AHBMASK_DSU                         (1 << 4)
#define MCLK_AHBMASK_NVMCTRL                     (1 << 6)
#define MCLK_AHBMASK_CMCC                        (1 << 8)
#define MCLK_AHBMASK_DMAC                        (1 << 9)
#define MCLK_AHBMASK_USB                         (1 << 10)
#define MCLK_AHBMASK_PAC                         (1 << 12)
#define MCLK_AHBMASK_QSPI                        (1 << 13)
#define MCLK_AHBMASK_GMAC                        (1 << 14)
#define MCLK_AHBMASK_SDHCn0                      (1 << 15)
#define MCLK_AHBMASK_SDHCn1                      (1 << 16)
#define MCLK_AHBMASK_CANn0                       (1 << 17)
#define MCLK_AHBMASK_CANn1                       (1 << 18)
#define MCLK_AHBMASK_ICM                         (1 << 19)
#define MCLK_AHBMASK_PUKCC                       (1 << 20)
#define MCLK_AHBMASK_QSPI2X                      (1 << 21)
#define MCLK_AHBMASK_NVMCTRL_SMEEPROM            (1 << 22)
#define MCLK_AHBMASK_NVMCTRL_CACHE               (1 << 23)
    uint32_t apbamask;
#define MCLK_APBAMASK_PAC                        (1 << 0)
#define MCLK_APBAMASK_PM                         (1 << 1)
#define MCLK_APBAMASK_MCLK                       (1 << 2)
#define MCLK_APBAMASK_RSTC                       (1 << 3)
#define MCLK_APBAMASK_OSCCTRL                    (1 << 4)
#define MCLK_APBAMASK_OSC32KCTRL                 (1 << 5)
#define MCLK_APBAMASK_SUPC                       (1 << 6)
#define MCLK_APBAMASK_GCLK                       (1 << 7)
#define MCLK_APBAMASK_WDT                        (1 << 8)
#define MCLK_APBAMASK_RTC                        (1 << 9)
#define MCLK_APBAMASK_EIC                        (1 << 10)
#define MCLK_APBAMASK_FREQM                      (1 << 11)
#define MCLK_APBAMASK_SERCOM0                    (1 << 12)
#define MCLK_APBAMASK_SERCOM1                    (1 << 13)
#define MCLK_APBAMASK_TC0                        (1 << 14)
#define MCLK_APBAMASK_TC1                        (1 << 15)
    uint32_t apbbmask;
#define MCLK_APBBMASK_USB                        (1 << 0)
#define MCLK_APBBMASK_DSU                        (1 << 1)
#define MCLK_APBBMASK_NVMCTRL                    (1 << 2)
#define MCLK_APBBMASK_PORT                       (1 << 4)
#define MCLK_APBBMASK_EVSYS                      (1 << 7)
#define MCLK_APBBMASK_SERCOM2                    (1 << 9)
#define MCLK_APBBMASK_SERCOM3                    (1 << 10)
#define MCLK_APBBMASK_TCC0                       (1 << 11)
#define MCLK_APBBMASK_TCC1                       (1 << 12)
#define MCLK_APBBMASK_TC2                        (1 << 13)
#define MCLK_APBBMASK_TC3                        (1 << 14)
    uint32_t apbcmask;
#define MCLK_APBCMASK_GMAC                       (1 << 2)
#define MCLK_APBCMASK_TCC2                       (1 << 3)
#define MCLK_APBCMASK_TCC3                       (1 << 4)
#define MCLK_APBCMASK_TC4                        (1 << 5)
#define MCLK_APBCMASK_TC5                        (1 << 6)
#define MCLK_APBCMASK_PDEC                       (1 << 7)
#define MCLK_APBCMASK_AC                         (1 << 8)
#define MCLK_APBCMASK_AES                        (1 << 9)
#define MCLK_APBCMASK_TRNG                       (1 << 10)
#define MCLK_APBCMASK_ICM                        (1 << 11)
#define MCLK_APBCMASK_QSPI                       (1 << 13)
#define MCLK_APBCMASK_CCL                        (1 << 14)
    uint32_t apbdmask;
#define MCLK_APBDMASK_SERCOM4                    (1 << 0)
#define MCLK_APBDMASK_SERCOM5                    (1 << 1)
#define MCLK_APBDMASK_SERCOM6                    (1 << 2)
#define MCLK_APBDMASK_SERCOM7                    (1 << 3)
#define MCLK_APBDMASK_TCC4                       (1 << 4)
#define MCLK_APBDMASK_TC6                        (1 << 5)
#define MCLK_APBDMASK_TC7                        (1 << 6)
#define MCLK_APBDMASK_ADC0                       (1 << 7)
#define MCLK_APBDMASK_ADC1                       (1 << 8)
#define MCLK_APBDMASK_DAC                        (1 << 9)
#define MCLK_APBDMASK_I2S                        (1 << 10)
#define MCLK_APBDMASK_PCC                        (1 << 11)
} __attribute__((packed)) mclk_t;


#define MCLK_BASE                                0x40000800
#define MCLK                                     ((volatile mclk_t *)MCLK_BASE)


typedef struct oscctrl
{
    uint8_t  evctrl;
#define OSCCTRL_EVCTRL_CFDEO0                    (1 << 0)
#define OSCCTRL_EVCTRL_CFDEO1                    (1 << 0)
    uint8_t  resvd_0x1[3];
    uint32_t intenclr;
#define OSCCTRL_INTENCLR_XOSCRDY0                (1 << 0)
#define OSCCTRL_INTENCLR_XOSCRDY1                (1 << 1)
#define OSCCTRL_INTENCLR_XOSCFAIL0               (1 << 2)
#define OSCCTRL_INTENCLR_XOSCFAIL1               (1 << 3)
#define OSCCTRL_INTENCLR_DFLLRDY                 (1 << 8)
#define OSCCTRL_INTENCLR_DFLLOOB                 (1 << 9)
#define OSCCTRL_INTENCLR_DFLLLCKF                (1 << 10)
#define OSCCTRL_INTENCLR_DFLLLCKC                (1 << 11)
#define OSCCTRL_INTENCLR_DFLLRCS                 (1 << 12)
#define OSCCTRL_INTENCLR_DPLLL0CKR               (1 << 16)
#define OSCCTRL_INTENCLR_CPLLL0CKF               (1 << 17)
#define OSCCTRL_INTENCLR_CPLLL0TO                (1 << 18)
#define OSCCTRL_INTENCLR_DPLLL0DRTO              (1 << 19)
#define OSCCTRL_INTENCLR_DPLLL1CKR               (1 << 24)
#define OSCCTRL_INTENCLR_CPLLL1CKF               (1 << 25)
#define OSCCTRL_INTENCLR_CPLLL1TO                (1 << 26)
#define OSCCTRL_INTENCLR_DPLLL1DRTO              (1 << 27)
    uint32_t intenset;
#define OSCCTRL_INTENSET_XOSCRDY0                (1 << 0)
#define OSCCTRL_INTENSET_XOSCRDY1                (1 << 1)
#define OSCCTRL_INTENSET_XOSCFAIL0               (1 << 2)
#define OSCCTRL_INTENSET_XOSCFAIL1               (1 << 3)
#define OSCCTRL_INTENSET_DFLLRDY                 (1 << 8)
#define OSCCTRL_INTENSET_DFLLOOB                 (1 << 9)
#define OSCCTRL_INTENSET_DFLLLCKF                (1 << 10)
#define OSCCTRL_INTENSET_DFLLLCKC                (1 << 11)
#define OSCCTRL_INTENSET_DFLLRCS                 (1 << 12)
#define OSCCTRL_INTENSET_DPLLL0CKR               (1 << 16)
#define OSCCTRL_INTENSET_CPLLL0CKF               (1 << 17)
#define OSCCTRL_INTENSET_CPLLL0TO                (1 << 18)
#define OSCCTRL_INTENSET_DPLLL0DRTO              (1 << 19)
#define OSCCTRL_INTENSET_DPLLL1CKR               (1 << 24)
#define OSCCTRL_INTENSET_CPLLL1CKF               (1 << 25)
#define OSCCTRL_INTENSET_CPLLL1TO                (1 << 26)
#define OSCCTRL_INTENSET_DPLLL1DRTO              (1 << 27)
    uint32_t intflag;
#define OSCCTRL_INTFLAG_XOSCRDY0                 (1 << 0)
#define OSCCTRL_INTFLAG_XOSCRDY1                 (1 << 1)
#define OSCCTRL_INTFLAG_XOSCFAIL0                (1 << 2)
#define OSCCTRL_INTFLAG_XOSCFAIL1                (1 << 3)
#define OSCCTRL_INTFLAG_DFLLRDY                  (1 << 8)
#define OSCCTRL_INTFLAG_DFLLOOB                  (1 << 9)
#define OSCCTRL_INTFLAG_DFLLLCKF                 (1 << 10)
#define OSCCTRL_INTFLAG_DFLLLCKC                 (1 << 11)
#define OSCCTRL_INTFLAG_DFLLRCS                  (1 << 12)
#define OSCCTRL_INTFLAG_DPLL0LCKR                (1 << 16)
#define OSCCTRL_INTFLAG_CPLL0LCKF                (1 << 17)
#define OSCCTRL_INTFLAG_CPLLL0TO                 (1 << 18)
#define OSCCTRL_INTFLAG_DPLLL0DRTO               (1 << 19)
#define OSCCTRL_INTFLAG_DPLL1LCKR                (1 << 24)
#define OSCCTRL_INTFLAG_CPLL1LCKF                (1 << 25)
#define OSCCTRL_INTFLAG_CPLLL1TO                 (1 << 26)
#define OSCCTRL_INTFLAG_DPLLL1DRTO               (1 << 27)
    uint32_t status;
#define OSCCTRL_STATUS_XOSCRDY0                  (1 << 0)
#define OSCCTRL_STATUS_XOSCRDY1                  (1 << 1)
#define OSCCTRL_STATUS_XOSCFAIL0                 (1 << 2)
#define OSCCTRL_STATUS_XOSCFAIL1                 (1 << 3)
#define OSCCTRL_STATUS_XOSCCKSW0                 (1 << 4)
#define OSCCTRL_STATUS_XOSCCKSW1                 (1 << 5)
#define OSCCTRL_STATUS_DFLLRDY                   (1 << 8)
#define OSCCTRL_STATUS_DFLLOOB                   (1 << 9)
#define OSCCTRL_STATUS_DFLLLCKF                  (1 << 10)
#define OSCCTRL_STATUS_DFLLLCKC                  (1 << 11)
#define OSCCTRL_STATUS_DFLLRCS                   (1 << 12)
#define OSCCTRL_STATUS_DPLL0LCKR                 (1 << 16)
#define OSCCTRL_STATUS_DPLL0LCKF                 (1 << 17)
#define OSCCTRL_STATUS_DPLLL0TO                  (1 << 18)
#define OSCCTRL_STATUS_DPLLL0DRTO                (1 << 19)
#define OSCCTRL_STATUS_DPLL1LCKR                 (1 << 24)
#define OSCCTRL_STATUS_DPLL1LCKF                 (1 << 25)
#define OSCCTRL_STATUS_DPLLL1TO                  (1 << 26)
#define OSCCTRL_STATUS_DPLLL1DRTO                (1 << 27)
    uint32_t xoscctrl[2];
#define OSCCTRL_XOSCCTRL_ENABLE                  (1 << 1)
#define OSCCTRL_XOSCCTRL_XTALEN                  (1 << 2)
#define OSCCTRL_XOSCCTRL_RUNSTDBY                (1 << 6)
#define OSCCTRL_XOSCCTRL_ONDEMAND                (1 << 7)
#define OSCCTRL_XOSCCTRL_LOWBUFGAIN              (1 << 8)
#define OSCCTRL_XOSCCTRL_IPTAT(val)              (((val) & 0x3) << 9)
#define OSCCTRL_XOSCCTRL_IPMULTI(val)            (((val) & 0xf) << 11)
#define OSCCTRL_XOSCCTRL_ENALC                   (1 << 15)
#define OSCCTRL_XOSCCTRL_CFDEN                   (1 << 16)
#define OSCCTRL_XOSCCTRL_SWBEN                   (1 << 17)
#define OSCCTRL_XOSCCTRL_STARTUP(val)            ((val & 0xff) << 20)
#define OSCCTRL_XOSCCTRL_CFDPRESC(val)           ((val & 0xff) << 24)
    uint8_t dfllctrla;
#define OSCCTRL_DFLLCTRLA_ENABLE                 (1 << 1)
#define OSCCTRL_DFLLCTRLA_RUNSTDBY               (1 << 6)
#define OSCCTRL_DFLLCTRLA_ONDEMAND               (1 << 7)
    uint8_t resvd_0x1d[3];
    uint8_t dfllctrlb;
#define OSCCTRL_DFLLCTRLB_MODE                   (1 << 0)
#define OSCCTRL_DFLLCTRLB_STABLE                 (1 << 1)
#define OSCCTRL_DFLLCTRLB_LLAW                   (1 << 2)
#define OSCCTRL_DFLLCTRLB_USBCRM                 (1 << 3)
#define OSCCTRL_DFLLCTRLB_CCDIS                  (1 << 4)
#define OSCCTRL_DFLLCTRLB_QLDIS                  (1 << 5)
#define OSCCTRL_DFLLCTRLB_BPLCKC                 (1 << 6)
#define OSCCTRL_DFLLCTRLB_WAITLOCK               (1 << 7)
    uint8_t resvd_0x21[3];
    uint32_t dfllval;
#define OSCCTRL_DFLLVAL_FINE(val)                (((val) & 0x3ff) << 0)
#define OSCCTRL_DFLLVAL_COARSE(val)              (((val) & 0x3f) << 10)
#define OSCCTRL_DFLLVAL_DIFF(val)                (((val) & 0xffff) << 16)
    uint32_t dfllmul;
#define OSCCTRL_DFLLMUL_MUL(val)                 (((val) & 0xffff) << 0)
#define OSCCTRL_DFLLMUL_FSTEP(val)               (((val) & 0x3ff) << 16)
#define OSCCTRL_DFLLMUL_CSTEP(val)               (((val) & 0x3f) << 26)
    uint8_t  dfllsync;
#define OSCCTRL_DFLLSYNC_ENABLE                  (1 << 1)
#define OSCCTRL_DFLLSYNC_DFLLCTRLB               (1 << 2)
#define OSCCTRL_DFLLSYNC_DFLLVAL                 (1 << 3)
#define OSCCTRL_DFLLSYNC_DFLLMUL                 (1 << 4)
    uint8_t  resvd_0x25[3];
    uint8_t  dpll0ctrla;
#define OSCCTRL_DPLL0CTRLA_ENABLE                (1 << 1)
#define OSCCTRL_DPLL0CTRLA_RUNSTDBY              (1 << 6)
#define OSCCTRL_DPLL0CTRLA_ONDEMAND              (1 << 7)
    uint8_t  resvd_0x29[3];
    uint32_t dpll0ratio;
#define OSCCTRL_DPLL0RATIO_LDR(val)              (((val) & 0xfff) << 0)
#define OSCCTRL_DPLL0RATIO_LDRFRAC(val)          (((val) & 0xf) << 16)
    uint32_t dpll0ctrlb;
#define OSCCTRL_DPLL0CTRLB_FILTER(val)           (((val) & 0xf) << 0)
#define OSCCTRL_DPLL0CTRLB_WUF                   (1 << 4)
#define OSCCTRL_DPLL0CTRLB_REFCLK_GCLK           (0x0 << 5)
#define OSCCTRL_DPLL0CTRLB_REFCLK_XOSC32K        (0x1 << 5)
#define OSCCTRL_DPLL0CTRLB_REFCLK_XOSC0          (0x2 << 5)
#define OSCCTRL_DPLL0CTRLB_REFCLK_XOSC1          (0x3 << 5)
#define OSCCTRL_DPLL0CTRLB_LTIME(val)            (((val) & 0x7) << 8)
#define OSCCTRL_DPLL0CTRLB_LBYPASS               (1 << 11)
#define OSCCTRL_DPLL0CTRLB_DCOFILTER(val)        ((val & 0x7) << 12)
#define OSCCTRL_DPLL0CTRLB_DCOEN                 (1 << 15)
#define OSCCTRL_DPLL0CTRLB_DIV(val)              (((val) & 0x3ff) << 16)
    uint32_t dpll0syncbusy;
#define OSCCTRL_DPLL0SYNCBUSY_ENABLE             (1 << 1)
#define OSCCTRL_DPLL0SYNCBUSY_DPLLRATIO          (1 << 2)
    uint32_t dpll0status;
#define OSCCTRL_DPLL0STATUS_LOCK                 (1 << 0)
#define OSCCTRL_DPLL0STATUS_CLKRDY               (1 << 1)
    uint8_t  dpll1ctrla;
#define OSCCTRL_DPLL1CTRLA_ENABLE                (1 << 1)
#define OSCCTRL_DPLL1CTRLA_RUNSTDBY              (1 << 6)
#define OSCCTRL_DPLL1CTRLA_ONDEMAND              (1 << 7)
    uint8_t  resvd_0x45[3];
    uint32_t dpll1ratio;
#define OSCCTRL_DPLL1RATIO_LDR(val)              (((val) & 0xfff) << 0)
#define OSCCTRL_DPLL1RATIO_LDRFRAC(val)          (((val) & 0xf) << 16)
    uint32_t dpll1ctrlb;
#define OSCCTRL_DPLL1CTRLB_FILTER(val)           (((val) & 0xf) << 0)
#define OSCCTRL_DPLL1CTRLB_WUF                   (1 << 4)
#define OSCCTRL_DPLL1CTRLB_REFCLK_XOSC32K        (0x0 << 5)
#define OSCCTRL_DPLL1CTRLB_REFCLK_XOSC           (0x1 << 5)
#define OSCCTRL_DPLL1CTRLB_REFCLK_GCLK           (0x2 << 5)
#define OSCCTRL_DPLL1CTRLB_LTIME(val)            (((val) & 0x7) << 8)
#define OSCCTRL_DPLL1CTRLB_LBYPASS               (1 << 11)
#define OSCCTRL_DPLL1CTRLB_DCOFILTER(val)        ((val & 0x7) << 12)
#define OSCCTRL_DPLL1CTRLB_DCOEN                 (1 << 15)
#define OSCCTRL_DPLL1CTRLB_DIV(val)              (((val) & 0x3ff) << 16)
    uint32_t dpll1syncbusy;
#define OSCCTRL_DPLL1SYNCBUSY_ENABLE            (1 << 1)
#define OSCCTRL_DPLL1SYNCBUSY_DPLLRATIO         (1 << 2)
    uint32_t dpll1status;
#define OSCCTRL_DPLL1STATUS_LOCK                (1 << 0)
#define OSCCTRL_DPLL1STATUS_CLKRDY              (1 << 1)
} __attribute__((packed)) oscctrl_t;


#define OSCCTRL_BASE                             0x40001000
#define OSCCTRL                                  ((volatile oscctrl_t *)OSCCTRL_BASE)

#define DFLLCTRL_COARSE_VAL                      (*((uint32_t *)0x00806020) >> 26)
#define DFLLVAL_USB_RECOVERY                     0xbb80


typedef struct xosc32k
{
    uint32_t intenclr;
#define XOSC32K_INTENCLR_XOSC32KRDY              (1 << 0)
#define XOSC32K_INTENCLR_XOSC32KFAIL             (1 << 2)
    uint32_t intenset;
#define XOSC32K_INTENSET_XOSC32KRDY              (1 << 0)
#define XOSC32K_INTENSET_XOSC32KFAIL             (1 << 2)
    uint32_t intflag;
#define XOSC32K_INTFLAG_XOSC32KRDY               (1 << 0)
#define XOSC32K_INTFLAG_XOSC32KFAIL              (1 << 2)
    uint32_t status;
#define XOSC32K_STATUS_XOSC32KRDY                (1 << 0)
#define XOSC32K_STATUS_XOSC32KFAIL               (1 << 2)
#define XOSC32K_STATUS_XOSC32KSW                 (1 << 3)
    uint8_t  rtcctrl;
#define XOSC32K_RTCCTRL_RTCSEL(val)              ((val & 0x7) << 0)
    uint8_t  resvd_0x11[3];
    uint16_t xosc32k;
#define XOSC32K_XOSC32K_ENABLE                   (1 << 1)
#define XOSC32K_XOSC32K_XTALEN                   (1 << 2)
#define XOSC32K_XOSC32K_EN32K                    (1 << 3)
#define XOSC32K_XOSC32K_EN1K                     (1 << 4)
#define XOSC32K_XOSC32K_RUNSTDBY                 (1 << 6)
#define XOSC32K_XOSC32K_ONDEMAND                 (1 << 7)
#define XOSC32K_XOSC32K_STARTUP(val)             ((val & 0x7) << 8)
#define XOSC32K_XOSC32K_WRTLOCK                  (1 << 12)
#define XOSC32K_XOSC32K_CGM(val)                 ((val & 0x3) << 13)
#define XOSC32K_XOSC32K_CGM_XT                   (0x1 << 13)
#define XOSC32K_XOSC32K_CGM_HS                   (0x2 << 13)
    uint8_t  cfdctrl;
#define XOSC32K_CFDCTRL_CFDEN                    (1 << 0)
#define XOSC32K_CFDCTRL_SWBACK                   (1 << 1)
#define XOSC32K_CFDCTRL_CFDPRESC                 (1 << 2)
    uint8_t  evctrl;
#define XOSC32K_EVCTRL_CFDEO                     (1 << 0)
    uint8_t  resvd_0x18[4];
    uint32_t osculp32k;
#define XOSC32K_OSCULP32K_CALIB(val)             ((val & 0x3f) << 0)
#define XOSC32K_OSCULP32K_WRTLOCK                (1 << 7)
} __attribute__ ((packed)) xosc32k_t;


#define XOSC32K                                  ((volatile xosc32k_t *)0x40001400)


typedef struct pm
{
    uint8_t ctrla;
#define PM_CTRLA_IORET                           (1 << 2)
    uint8_t sleepcfg;
#define PM_SLEEPCFG_MODE(val)                    ((val & 0x7) << 0)
    uint8_t resvd_0x02[2];
    uint8_t intenclr;
#define PM_INTENCLR_SLEEPRDY                     (1 << 0)
    uint8_t intenset;
#define PM_INTENSET_SLEEPRDY                     (1 << 0)
    uint8_t intflag;
#define PM_INTFLAG_SLEEPRDY                      (1 << 0)
    uint8_t resvd_0x7;
    uint8_t stdbycfg;
#define PM_STDBYCFG_RAMCFG(val)                  ((val & 0x3) << 0)
#define PM_STDBYCFG_FASTWKUP(val)                ((val & 0x3) << 4)
    uint8_t hibcfg;
#define PM_HIBCFG_RAMCFG(val)                    ((val & 0x3) << 0)
#define PM_HIBCFG_BRAMCFG(val)                   ((val & 0x3) << 2)
    uint8_t bkupcfg;
#define PM_BKUPCFG_BRAMCFG(val)                  ((val & 0x3) << 0)
} __attribute__((packed)) pm_t;

#define PM_BASE                                  0x40000400
#define PM                                       ((volatile pm_t *)PM_BASE)


#endif /* __AT91SAMD53__ */


void gclk_setup(uint8_t clknum, uint8_t src, uint16_t div);
void gclk_peripheral_enable(uint8_t clknum, uint8_t peripheral);
void gclk_peripheral_disable(uint8_t clknum, uint8_t peripheral);

#endif /* __SAML_CLOCKS_H__ */
