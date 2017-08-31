/*
 * sam4_adc.h
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

#ifndef __SAM4S_ADC_H__
#define __SAM4S_ADC_H__

#if defined(__AT91SAM4S__)

#include "sam4_pdc.h"


#define CONSOLE_CMD_ADC                          \
    {                                            \
        .cmdstr = "adc",                         \
        .callback = cmd_adc,                     \
        .usage = "  adc\r\n", \
        .help =  "  Display ADC Power Voltage and Temperatures.\r\n",  \
    }


typedef struct adc
{
    uint32_t cr;
#define ADC_CR_SWRST                              (1 << 0)
#define ADC_CR_START                              (1 << 1)
#define ADC_CR_AUTOCAL                            (1 << 3)
    uint32_t mr;
#define ADC_MR_TRGEN                              (1 << 0)
#define ADC_MR_TRIG0                              ((0 & 0x7) << 1)
#define ADC_MR_TRIG1                              ((1 & 0x7) << 1)
#define ADC_MR_TRIG2                              ((2 & 0x7) << 1)
#define ADC_MR_TRIG3                              ((3 & 0x7) << 1)
#define ADC_MR_TRIG4                              ((4 & 0x7) << 1)
#define ADC_MR_TRIG5                              ((5 & 0x7) << 1)
#define ADC_MR_LOWRES                             (1 << 4)
#define ADC_MR_SLEEP                              (1 << 5)
#define ADC_MR_FWUP                               (1 << 6)
#define ADC_MR_FREERUN                            (1 << 7)
#define ADC_MR_PRESCAL(val)                       ((val & 0xff) << 8)
#define ADC_MR_STARTUP(val)                       ((val & 0x0f) << 16)
#define ADC_MR_SETTLING(val)                      ((val & 0x03) << 20)
#define ADC_MR_ANACH                              (1 << 23)
#define ADC_MR_TRACKTIM(val)                      ((val & 0x0f) << 24)
#define ADC_MR_TRANSFER(val)                      ((val & 0x03) << 28)
#define ADC_MR_USEQ                               (1 << 31)
    uint32_t seqr1;
    uint32_t seqr2;
    uint32_t cher;
    uint32_t chdr;
#define ADC_CHDR_DISABLE_ALL                      (0xffff < 0)
    uint32_t chsr;
    uint32_t resvd_0x1c;
    uint32_t lcdr;
    uint32_t ier;
#define ADC_IER_CHANNEL_MASK(val)                 ((val & 0xffff) << 0)
#define ADC_IER_EOCAL                             (1 << 23)
#define ADC_IER_DRDY                              (1 << 24)
#define ADC_IER_GOVRE                             (1 << 25)
#define ADC_IER_COMPE                             (1 << 26)
#define ADC_IER_ENDRX                             (1 << 27)
#define ADC_IER_RXBUF                             (1 << 28)
    uint32_t idr;
#define ADC_IDR_CHANNEL_MASK(val)                 ((val & 0xffff) << 0)
#define ADC_IDR_EOCAL                             (1 << 23)
#define ADC_IDR_DRDY                              (1 << 24)
#define ADC_IDR_GOVRE                             (1 << 25)
#define ADC_IDR_COMPE                             (1 << 26)
#define ADC_IDR_ENDRX                             (1 << 27)
#define ADC_IDR_RXBUF                             (1 << 28)
    uint32_t imr;
    uint32_t isr;
    uint32_t resvd_0x34[2];
    uint32_t over;
    uint32_t emr;
    uint32_t cwr;
    uint32_t cgr;
    uint32_t cor;
#define ADC_COR_DIFF_MASK(val)                    ((val & 0xffff) << 16)
    uint32_t cdr[16];
    uint32_t resvd_0x90;
    uint32_t acr;
#define ADC_ACR_TSON                              (1 << 4)
#define ADC_ACR_IBCTL(val)                        ((val & 0x3) << 8)
    uint32_t resvd_0x98[19];
    uint32_t wpmr;
    uint32_t wpsr;
    uint32_t resvd_0xec[5];
    pdc_t pdc;
} __attribute__ ((packed)) adc_t;

#define ADC                                       ((volatile adc_t *)0x40038000)

#define ADC_AVERAGE_SAMPLES                       512

typedef struct adc_calc_temp_t
{
    int32_t offset;
    uint16_t *table;
    uint16_t table_size;
} adc_calc_temp_t;

typedef struct adc_calc_current_lt6105
{
    uint32_t mvref;
    int32_t offset;
    uint32_t rin;
    uint32_t rout;
    uint32_t rsense_mohms;
} adc_calc_current_lt6105_t;

typedef struct adc_calc_current_ltc6102
{
    uint32_t mvref;
    int32_t offset;
    uint32_t rin_mohms;
    uint32_t rout;
    uint32_t rsense_mohms;
} adc_calc_current_ltc6102_t;

typedef struct adc_calc_voltage_direct
{
    uint32_t mvref;
    int32_t offset;
} adc_calc_voltage_direct_t;

typedef struct adc_calc_voltage_divider
{
    uint32_t mvref;
    uint32_t r1;
    uint32_t r2;
    int32_t offset;
} adc_calc_voltage_divider_t;


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
#define ADC_DRV_FLAGS_DIFF                         (1 << 0)
#define ADC_DRV_FLAGS_CALIBRATE                    (1 << 1)
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

void adc_init(void);
int adc_start(adc_queue_entry_t *entry);
int adc_finished(adc_queue_entry_t *entry);

/* Calculation Handlers */
void adc_calc_temp_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                  uint64_t total, uint32_t min,
                                  uint32_t max, uint64_t count, void *arg);
void adc_calc_divider_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                     uint64_t total, uint32_t min,
                                     uint32_t max, uint64_t count, void *arg);
void adc_calc_direct_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                    uint64_t total, uint32_t min,
                                    uint32_t max, uint64_t count, void *arg);
void adc_calc_lt6105_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                    uint64_t total, uint32_t min,
                                    uint32_t max, uint64_t count, void *arg);
void adc_calc_ltc6102_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                    uint64_t total, uint32_t min,
                                    uint32_t max, uint64_t count, void *arg);

#endif /*  defined(__AT91SAM4S__) */

#endif /* __SAM4S_ADC_H__ */

