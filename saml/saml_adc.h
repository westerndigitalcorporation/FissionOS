/*
 * saml_adc.h
 *
 *
 * Copyright (c) 2017 Western Digital Corporation or its affiliates.
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

#ifndef __SAML_ADC_H__
#define __SAML_ADC_H__


#include <workqueue.h>
#include <adc_calc.h>

#if defined(__ATSAMD53__)


#define CONSOLE_CMD_ADC                          \
    {                                            \
        .cmdstr = "adc",                         \
        .callback = cmd_adc,                     \
        .usage = "  adc\r\n", \
        .help =  "  Display ADC Power Voltage and Temperatures.\r\n",  \
    }


typedef struct adc
{
    uint16_t ctrla;
#define ADC_CTRLA_SWRST                          (1 << 0)
#define ADC_CTRLA_ENABLE                         (1 << 1)
#define ADC_CTRLA_DUALSEL(val)                   ((val & 0x3) << 3)
#define ADC_CTRLA_SLAVEEN                        (1 << 5)
#define ADC_CTRLA_RUNSTDBY                       (1 << 6)
#define ADC_CTRLA_ONDEMAND                       (1 << 7)
#define ADC_CTRLA_PRESCALER(val)                 ((val & 0x7) << 8)
#define ADC_CTRLA_PRESCALER_DIV2                 (0 << 8)
#define ADC_CTRLA_PRESCALER_DIV4                 (1 << 8)
#define ADC_CTRLA_PRESCALER_DIV8                 (2 << 8)
#define ADC_CTRLA_PRESCALER_DIV16                (3 << 8)
#define ADC_CTRLA_PRESCALER_DIV32                (4 << 8)
#define ADC_CTRLA_PRESCALER_DIV64                (5 << 8)
#define ADC_CTRLA_PRESCALER_DIV128               (6 << 8)
#define ADC_CTRLA_PRESCALER_DIV256               (7 << 8)
#define ADC_CTRLA_R2R                            (1 << 15)
    uint8_t  evctrl;
#define ADC_EVCTRL_FLUSHEI                       (1 << 0)
#define ADC_EVCTRL_STARTEI                       (1 << 1)
#define ADC_EVCTRL_FLUSHINV                      (1 << 2)
#define ADC_EVCTRL_STARTINV                      (1 << 3)
#define ADC_EVCTRL_RESRDYEO                      (1 << 4)
#define ADC_EVCTRL_WINMONEO                      (1 << 5)
    uint8_t  dbgctrl;
#define ADC_DBGCTRL_DBGRUN                       (1 << 0)
    uint16_t inputctrl;
#define ADC_INPUTCTRL_MUXPOS(val)                ((val & 0x1f) << 0)
#define ADC_INPUTCTRL_DIFFMODE                   (1 << 7)
#define ADC_INPUTCTRL_MUXNEG(val)                ((val & 0x1f) << 8)
#define ADC_INPUTCTRL_DEQSTOP                    (1 << 8)
    uint16_t ctrlb;
#define ADC_CTRLB_LEFTADJ                        (1 << 0)
#define ADC_CTRLB_FREERUN                        (1 << 1)
#define ADC_CTRLB_CORREN                         (1 << 2)
#define ADC_CTRLB_RESSEL(val)                    ((val & 0x3) << 3)
#define ADC_CTRLB_WINMODE(val)                   ((val & 0x7) << 8)
#define ADC_CTRLB_WINSS                          (1 << 11)
    uint8_t  refctrl;
#define ADC_REFCTRL_REFSEL(val)                  ((val & 0xf) << 0)
#define ADC_REFCTRL_REFCOMP                      (1 << 7)
    uint8_t  resvd_0x09;
    uint8_t  avgctrl;
#define ADC_AVGCTRL_SAMPLENUM(val)               ((val & 0xf) << 0)
#define ADC_AVGCTRL_ADJRES(val)                  ((val & 0x7) << 4)
    uint8_t  sampctrl;
#define ADC_SAMPCTRL_SAMPLEN(val)                ((val & 0x3f) << 0)
#define ADC_SAMPCTRL_OFFCOMP                     (1 << 7)
    uint16_t winlt;
    uint16_t winut;
    uint16_t gaincorr;
    uint16_t offsetcorr;
    uint8_t  swtrig;
#define ADC_SWTRIG_FLUSH                         (1 << 0)
#define ADC_SWTRIG_START                         (1 << 1)
    uint8_t  resvd_0x15[0x17];
    uint8_t  intenclr;
#define ADC_INTENCLR_RESRDY                      (1 << 0)
#define ADC_INTENCLR_OVERRUN                     (1 << 1)
#define ADC_INTENCLR_WINMON                      (1 << 2)
    uint8_t  intenset;
#define ADC_INTENSET_RESRDY                      (1 << 0)
#define ADC_INTENSET_OVERRUN                     (1 << 1)
#define ADC_INTENSET_WINMON                      (1 << 2)
    uint8_t  intflag;
#define ADC_INTFLAG_RESRDY                       (1 << 0)
#define ADC_INTFLAG_OVERRUN                      (1 << 1)
#define ADC_INTFLAG_WINMON                       (1 << 2)
    uint8_t  status;
#define ADC_STATUS_ADCBUSY                       (1 << 0)
#define ADC_STATUS_WCC(val)                      ((val & 0x3f) << 2)
    uint32_t syncbusy;
#define ADC_SYNCBUSY_SWRST                       (1 << 0)
#define ADC_SYNCBUSY_ENABLE                      (1 << 1)
#define ADC_SYNCBUSY_INPUTCTRL                   (1 << 2)
#define ADC_SYNCBUSY_CTRLB                       (1 << 3)
#define ADC_SYNCBUSY_REFCTRL                     (1 << 4)
#define ADC_SYNCBUSY_AVGCTRL                     (1 << 5)
#define ADC_SYNCBUSY_SAMPCTRL                    (1 << 6)
#define ADC_SYNCBUSY_WINLT                       (1 << 7)
#define ADC_SYNCBUSY_WINUT                       (1 << 8)
#define ADC_SYNCBUSY_GAINCORR                    (1 << 9)
#define ADC_SYNCBUSY_OFFSETCORR                  (1 << 10)
#define ADC_SYNCBUSY_SWTRIG                      (1 << 11)
#define ADC_SYNCBUSY_RBSSW                       (1 << 31)
    uint32_t dseqdata;
    uint32_t dseqctrl;
#define ADC_DSEQCTRL_INPUTCTRL                   (1 << 0)
#define ADC_DSEQCTRL_CTRLB                       (1 << 1)
#define ADC_DSEQCTRL_REFCTRL                     (1 << 2)
#define ADC_DSEQCTRL_AVGCTRL                     (1 << 3)
#define ADC_DSEQCTRL_SAMPCTRL                    (1 << 4)
#define ADC_DSEQCTRL_WINLT                       (1 << 5)
#define ADC_DSEQCTRL_WINUT                       (1 << 6)
#define ADC_DSEQCTRL_GAINCORR                    (1 << 7)
#define ADC_DSEQCTRL_OFFSETCORR                  (1 << 8)
#define ADC_DSEQCTRL_AUTOSTART                   (1 << 31)
    uint32_t dseqstat;
#define ADC_DSEQSTAT_INPUTCTRL                   (1 << 0)
#define ADC_DSEQSTAT_CTRLB                       (1 << 1)
#define ADC_DSEQSTAT_REFCTRL                     (1 << 2)
#define ADC_DSEQSTAT_AVGCTRL                     (1 << 3)
#define ADC_DSEQSTAT_SAMPCTRL                    (1 << 4)
#define ADC_DSEQSTAT_WINLT                       (1 << 5)
#define ADC_DSEQSTAT_WINUT                       (1 << 6)
#define ADC_DSEQSTAT_GAINCORR                    (1 << 7)
#define ADC_DSEQSTAT_OFFSETCORR                  (1 << 8)
#define ADC_DSEQSTAT_BUSY                        (1 << 31)
    uint16_t result;
    uint16_t resvd_0x42;
    uint16_t ress;
    uint16_t resvd_0x46;
    uint16_t calib;
#define ADC_CALIB_BIASCOMP(val)                  ((val & 0x7) << 0)
#define ADC_CALIB_BIASR2R(val)                   ((val & 0x7) << 4)
#define ADC_CALIB_BIASREFBUF(val)                ((val & 0x7) << 8)
} __attribute__ ((packed)) adc_t;

#define ADC0                                     ((volatile adc_t *)0x43001c00)
#define ADC1                                     ((volatile adc_t *)0x43002000)

#define ADC_COUNT                                2
#define ADC_AVERAGE_SAMPLES                      1


struct adc_drv;
typedef void (*adc_calc_callback_t)(struct adc_drv *adc, uint32_t *value,
              uint64_t total, uint32_t min, uint32_t max, uint64_t count, void *arg);
typedef void (*adc_setup_callback_t)(struct adc_drv *adc, void *arg);
typedef void (*adc_complete_t)(struct adc_drv **adc, uint32_t count, void *arg);

typedef struct adc_drv
{
    uint32_t channel;
    adc_calc_callback_t calc;
    void *calc_arg;
    adc_setup_callback_t setup;
    void *setup_arg;
    uint32_t *value;
    uint32_t flags;
    uint32_t gain;
    uint32_t total;
} adc_drv_t;

typedef struct adc_queue_entry
{
    struct adc_queue_entry *next;
    struct adc_queue_entry *prev;
    adc_complete_t complete;
    void *complete_arg;
    uint16_t count;
    adc_drv_t **adcs;
} adc_queue_entry_t;

typedef struct adc_desc
{
    volatile adc_t *adc;
    volatile uint32_t state;
    uint32_t active;
    uint32_t count;
    uint16_t conversion_results[1];
    adc_queue_entry_t queue_head;
    workqueue_t wq;
} adc_desc_t; 

int adc_init(int adcnum, adc_desc_t *desc);
int adc_start(adc_desc_t *desc, adc_queue_entry_t *entry);
int adc_finished(adc_queue_entry_t *entry);


#endif /*  defined(__ATSAMD53__) */

#endif /* __SAML_ADC_H__ */

