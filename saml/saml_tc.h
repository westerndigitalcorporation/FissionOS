/*
 * saml_tc.h
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


#ifndef __SAML_TC_H__
#define __SAML_TC_H__

typedef struct tc
{
    uint32_t ctrla;
#define TC_CTRLA_SWRST                           (1 << 0)
#define TC_CTRLA_ENABLE                          (1 << 1)
#define TC_CTRLA_MODE_COUNT8                     (0x1 << 2)
#define TC_CTRLA_MODE_COUNT16                    (0x0 << 2)
#define TC_CTRLA_MODE_COUNT32                    (0x2 << 2)
#define TC_CTRLA_PRESCSYNC_GCLK                  (0x0 << 4)
#define TC_CTRLA_PRESCSYNC_PRESC                 (0x1 << 4)
#define TC_CTRLA_PRESCSYNC_RESYNC                (0x2 << 4)
#define TC_CTRLA_RUNSTDBY                        (1 << 6)
#define TC_CTRLA_ONDEMAND                        (1 << 7)
#define TC_CTRLA_PRESCALER_DIV1                  (0x0 << 8)
#define TC_CTRLA_PRESCALER_DIV2                  (0x1 << 8)
#define TC_CTRLA_PRESCALER_DIV4                  (0x2 << 8)
#define TC_CTRLA_PRESCALER_DIV8                  (0x3 << 8)
#define TC_CTRLA_PRESCALER_DIV16                 (0x4 << 8)
#define TC_CTRLA_PRESCALER_DIV64                 (0x5 << 8)
#define TC_CTRLA_PRESCALER_DIV256                (0x6 << 8)
#define TC_CTRLA_PRESCALER_DIV1024               (0x7 << 8)
#define TC_CTRLA_ALOCK                           (1 << 11)
#define TC_CTRLA_CAPTEN0                         (1 << 16)
#define TC_CTRLA_CAPTEN1                         (1 << 17)
#define TC_CTRLA_COPEN0                          (1 << 20)
#define TC_CTRLA_COPEN1                          (1 << 21)
    uint8_t ctrlbclr;
#define TC_CTRLBCLR_DIR                          (1 << 0)
#define TC_CTRLBCLR_LUPD                         (1 << 1)
#define TC_CTRLBCLR_ONESHOT                      (1 << 2)
#define TC_CTRLBCLR_CMD_CLEAR                    (1 << 5)
    uint8_t ctrlbset;
#define TC_CTRLBSET_DIR                          (1 << 0)
#define TC_CTRLBSET_LUPD                         (1 << 1)
#define TC_CTRLBSET_ONESHOT                      (1 << 2)
#define TC_CTRLBSET_CMD_NONE                     (0x0 << 5)
#define TC_CTRLBSET_CMD_RETRIGGER                (0x1 << 5)
#define TC_CTRLBSET_CMD_STOP                     (0x2 << 5)
#define TC_CTRLBSET_CMD_UPDATE                   (0x3 << 5)
#define TC_CTRLBSET_CMD_READSYNC                 (0x4 << 5)
    uint16_t evctrl;
#define TC_EVCTRL_EVACT_OFF                      (0x0 << 0)
#define TC_EVCTRL_EVACT_RETRIGGER                (0x1 << 0)
#define TC_EVCTRL_EVACT_COUNT                    (0x2 << 0)
#define TC_EVCTRL_EVACT_START                    (0x3 << 0)
#define TC_EVCTRL_EVACT_STAMP                    (0x4 << 0)
#define TC_EVCTRL_EVACT_PPW                      (0x5 << 0)
#define TC_EVCTRL_EVACT_PWP                      (0x6 << 0)
#define TC_EVCTRL_EVACT_PW                       (0x7 << 0)
#define TC_EVCTRL_EVACT_TCINV                    (1 << 4)
#define TC_EVCTRL_EVACT_TCEI                     (1 << 5)
#define TC_EVCTRL_EVACT_OVFEO                    (1 << 8)
#define TC_EVCTRL_EVACT_MCEO0                    (1 << 12)
#define TC_EVCTRL_EVACT_MCEO1                    (1 << 13)
    uint8_t intenclr;
#define TC_INTENCLR_OVF                          (1 << 0)
#define TC_INTENCLR_ERR                          (1 << 1)
#define TC_INTENCLR_MC0                          (1 << 4)
#define TC_INTENCLR_MC1                          (1 << 5)
    uint8_t intenset;
#define TC_INTENSET_OVF                          (1 << 0)
#define TC_INTENSET_ERR                          (1 << 1)
#define TC_INTENSET_MC0                          (1 << 4)
#define TC_INTENSET_MC1                          (1 << 5)
    uint8_t intflag;
#define TC_INTENFLAG_OVF                         (1 << 0)
#define TC_INTENFLAG_ERR                         (1 << 1)
#define TC_INTENFLAG_MC0                         (1 << 4)
#define TC_INTENFLAG_MC1                         (1 << 5)
    uint8_t status;
#define TC_STATUS_STOP                           (1 << 0)
#define TC_STATUS_SLAVE                          (1 << 1)
#define TC_STATUS_PERBUFV                        (1 << 3)
#define TC_STATUS_CCBUFV0                        (1 << 4)
#define TC_STATUS_CCBUFV1                        (1 << 5)
    uint8_t wave;
#define TC_WAVE_NFRQ                             (0x0 << 0)
#define TC_WAVE_MFRQ                             (0x1 << 0)
#define TC_WAVE_NPWM                             (0x2 << 0)
#define TC_WAVE_MPWM                             (0x3 << 0)
    uint8_t drvctrl;
#define TC_DRVCTRL_INVEN0                        (1 << 0)
#define TC_DRVCTRL_INVEN1                        (1 << 1)
    uint8_t resvd_0x0e;
    uint8_t dbgctrl;
#define TC_DBGCTRL_DBGRUN                        (1 << 0)
    uint32_t syncbusy;
#define TC_SYNCBUSY_SWRST                        (1 << 0)
#define TC_SYNCBUSY_ENABLE                       (1 << 1)
#define TC_SYNCBUSY_CTRLB                        (1 << 2)
#define TC_SYNCBUSY_STATUS                       (1 << 3)
#define TC_SYNCBUSY_COUNT                        (1 << 4)
#define TC_SYNCBUSY_PER                          (1 << 5)
#define TC_SYNCBUSY_CC0                          (1 << 6)
#define TC_SYNCBUSY_CC1                          (1 << 7)
    uint16_t count;
    uint16_t resvd_0x16;
    uint32_t resvd_0x18;
    uint16_t cc0;
    uint16_t cc1;
    uint8_t resvd_0x20[16];
    uint16_t ccbuf0;
    uint16_t ccbuf1;
    uint8_t resvd_0x34[12];
} __attribute__((packed)) tc_t;


#if defined(__AT91SAML21__)
#define TC0                                      ((volatile tc_t *)0x42002000)
#define TC1                                      ((volatile tc_t *)0x42002400)
#define TC2                                      ((volatile tc_t *)0x42002800)
#define TC3                                      ((volatile tc_t *)0x42002c00)
#define TC4                                      ((volatile tc_t *)0x43000800)
#endif /* __AT91SAML21__ */

#if defined(__AT91SAMD20__)
#define TC0                                      ((volatile tc_t *)0x42002000)
#define TC1                                      ((volatile tc_t *)0x42002400)
#define TC2                                      ((volatile tc_t *)0x42002800)
#define TC3                                      ((volatile tc_t *)0x42002c00)
#define TC4                                      ((volatile tc_t *)0x42003000)
#define TC5                                      ((volatile tc_t *)0x42003400)
#define TC6                                      ((volatile tc_t *)0x42003800)
#define TC7                                      ((volatile tc_t *)0x42003c00)
#endif /* __AT91SAMD20__ */

void tc_pwm_init(volatile tc_t *tc, uint32_t prescale_flag,
                 uint8_t invert, uint16_t duty);
void tc_disable(volatile tc_t *tc);
void tc_pwm_duty(volatile tc_t *tc, uint16_t duty);


#endif /* __SAML_TC_H__ */
