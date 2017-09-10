/*
 * adc_calc.h
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
#ifndef __ADC_CALC_H__
#define __ADC_CALC_H__

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

typedef struct adc_drv adc_drv_t;
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

#endif /* __ADC_CALC_H__ */
