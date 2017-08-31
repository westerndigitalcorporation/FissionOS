/*
 * sam4_dac.h
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


#ifndef __SAM4S_DAC_H__
#define __SAM4S_DAC_H__

typedef struct dac
{
    uint32_t cr;
#define DAC_CR_SWRST                             (1 << 0)
    uint32_t mr;
#define DAC_MR_TRGEN                             (1 << 0)
#define DAC_MR_TRGSEL_EXTERN                     ((0 & 0x07) << 1)
#define DAC_MR_TRGSEL_TC0                        ((1 & 0x07) << 1)
#define DAC_MR_TRGSEL_TC1                        ((2 & 0x07) << 1)
#define DAC_MR_TRGSEL_TC2                        ((3 & 0x07) << 1)
#define DAC_MR_TRGSEL_PWM0                       ((4 & 0x07) << 1)
#define DAC_MR_TRGSEL_PWM1                       ((5 & 0x07) << 1)
#define DAC_MR_WORD                              (1 << 4)
#define DAC_MR_SLEEP                             (1 << 5)
#define DAC_MR_FASTWKUP                          (1 << 6)
#define DAC_MR_REFRESH(val)                      ((val & 0xff) << 8)
#define DAC_MR_USER_SEL_CHANNEL1                 ((1 & 0x03) << 16)
#define DAC_MR_TAG                               (1 << 20)
#define DAC_MR_MAXS                              (1 << 21)
#define DAC_MR_STARTUP(val)                      ((val & 0x3f) << 24)
    uint32_t resvd_0x08[2];
    uint32_t cher;
#define DAC_CHER_CH0                             (1 << 0)
#define DAC_CHER_CH1                             (1 << 1)
    uint32_t chdr;
#define DAC_CHDR_CH0                             (1 << 0)
#define DAC_CHDR_CH1                             (1 << 1)
    uint32_t chsr;
#define DAC_CHSR_CH0                             (1 << 0)
#define DAC_CHSR_CH1                             (1 << 1)
    uint32_t resvd_0x1c;
    uint32_t cdr;
#define DAC_CDR_CH0(val)                         (((val & 0xfff) << 0) | (0 << 12))
#define DAC_CDR_CH1(val)                         (((val & 0xfff) << 16) | (1 << 28))
    uint32_t ier;
#define DAC_IER_TXRDY                            (1 << 0)
#define DAC_IER_EOC                              (1 << 1)
#define DAC_IER_ENDTX                            (1 << 2)
#define DAC_IER_TXBUFE                           (1 << 3)
    uint32_t idr;
#define DAC_IDR_TXRDY                            (1 << 0)
#define DAC_IDR_EOC                              (1 << 1)
#define DAC_IDR_ENDTX                            (1 << 2)
#define DAC_IDR_TXBUFE                           (1 << 3)
    uint32_t imr;
#define DAC_IMR_TXRDY                            (1 << 0)
#define DAC_IMR_EOC                              (1 << 1)
#define DAC_IMR_ENDTX                            (1 << 2)
#define DAC_IMR_TXBUFE                           (1 << 3)
    uint32_t isr;
#define DAC_ISR_TXRDY                            (1 << 0)
#define DAC_ISR_EOC                              (1 << 1)
#define DAC_ISR_ENDTX                            (1 << 2)
#define DAC_ISR_TXBUFE                           (1 << 3)
    uint32_t resvd_0x34[24];
    uint32_t acr;
#define DAC_ACR_IBCTLCH0(val)                    ((val & 0x3) << 0)
#define DAC_ACR_IBCTLCH1(val)                    ((val & 0x3) << 2)
#define DAC_ACR_IBCTLDACCORE(val)                ((val & 0x3) << 8)
    uint32_t resvd_0x98[19];
    uint32_t wpmr;
    uint32_t wpsr;
} __attribute__ ((packed)) dac_t;

#if defined(__AT91SAM4S__)
#define DAC                                      ((volatile dac_t *)0x4003c000)
#elif defined(__AT91SAM4E__)
#define DAC                                      ((volatile dac_t *)0x400b8000)
#endif

// From the SAM4S electrical specifications
#define MAX_DAC_mV(VREF_mV)                       ((5 * VREF_mV) / 6)
#define MIN_DAC_mV(VREF_mV)                       ((1 * VREF_mV) / 6)
#define DAC_RANGE_mV(VREF_mV)                     (MAX_DAC_mV(VREF_mV) - MIN_DAC_mV(VREF_mV))
#define MAX_DAC_VALUE                             ((1 << 12) - 1)      //< 12-bit DAC

#define DAC_mV_TO_VALUE(VREF_mV, mV)              (((mV - MIN_DAC_mV(VREF_mV)) * MAX_DAC_VALUE) / \
                                                   DAC_RANGE_mV(VREF_mV))

#define DAC_CHANNELS                              2

typedef void (*dac_complete_t)(void *arg);
typedef struct dac_drv
{
    dac_complete_t complete;
    void *arg;
} dac_drv_t;

void dac_init(dac_drv_t *dacs);
void dac_enable(uint8_t channel);
void dac_disable(uint8_t channel);
int dac_set(uint8_t channel, uint16_t value);

#endif /* __SAM4S_DAC_H__ */
