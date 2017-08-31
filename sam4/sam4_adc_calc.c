/*
 * sam4_adc_calc.c
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


#include <stdint.h>
#include <stddef.h>

#include "sam4_adc.h"


#if defined(__AT91SAM4S__)

/**
 * Compute the temperature in 1/10 deg C given a sample, and a table of samples
 * at 1 degree intervals starting at 0 where the temperature matches the index.
 * Interpolate the least significant tenths by simple ratio between tables
 * entries.
 */
void adc_calc_temp_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                  uint64_t total, uint32_t min,
                                  uint32_t max, uint64_t count,
                                  void *arg)
{
    adc_calc_temp_t *calc = (adc_calc_temp_t *)arg;
    uint32_t sample = total / count;
    int i;

    // Limit the calculation to 0 degrees C.
    if ((sample + calc->offset) > calc->table[0])
    {
        sample = calc->table[0] - calc->offset;
    }

    for (i = 1; i < calc->table_size; i++)
    {
        if ((sample + calc->offset) > calc->table[i])
        {
            uint16_t range = calc->table[i - 1] - calc->table[i];
            uint32_t s = (sample + calc->offset) - calc->table[i];
            uint32_t d = 10 - ((10 * s) / range);        // Compute the ratio from 0 - 10

            *value = (10 * (i - 1)) + d;  // Return unit in 1/10 deg C

            return;
        }
    }
}

/**
 * Calculate the voltage on the pin given the sample and the reference voltage.
 */
void adc_calc_direct_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                    uint64_t total, uint32_t min,
                                    uint32_t max, uint64_t count,
                                    void *arg)
{
    uint32_t sample = total / count;

    adc_calc_voltage_direct_t *calc = (adc_calc_voltage_direct_t *)arg;

    *value = (((sample + calc->offset) * calc->mvref) / (1 << 12));
}

/**
 * Calculate the current given a output voltage and resistor values from a
 * LT6105 current sense amplifier.
 */
void adc_calc_lt6105_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                    uint64_t total, uint32_t min,
                                    uint32_t max, uint64_t count,
                                    void *arg)
{
    adc_calc_current_lt6105_t *calc = (adc_calc_current_lt6105_t *)arg;
    uint32_t sample = total / count;
    uint32_t amplified_mv;

    amplified_mv = (((sample + calc->offset) * calc->mvref) / (1 << 12));

    *value = ((amplified_mv * (calc->rin * 1000)) / (calc->rsense_mohms * (calc->rout)));
}

/**
 * Calculate the current given a output voltage and resistor values from a
 * LTC6102 current sense amplifier.
 */
void adc_calc_ltc6102_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                     uint64_t total, uint32_t min,
                                     uint32_t max, uint64_t count,
                                     void *arg)
{
    adc_calc_current_ltc6102_t *calc = (adc_calc_current_ltc6102_t *)arg;
    uint32_t sample = total / count;
    uint32_t amplified_mv;

    amplified_mv = (((sample + calc->offset) * calc->mvref) / (1 << 12));

    *value = ((amplified_mv * calc->rin_mohms) / (calc->rout * calc->rsense_mohms));
}

/**
 * Calculate original voltage given a sample from a voltage divider using the
 * reference voltage and both resistors (r1 and r2) of the divider circuit.
 */
void adc_calc_divider_12bit_unsigned(adc_drv_t *drv, uint32_t *value,
                                     uint64_t total, uint32_t min,
                                     uint32_t max, uint64_t count,
                                     void *arg)
{
    adc_calc_voltage_divider_t *calc = (adc_calc_voltage_divider_t *)arg;
    uint32_t sample = total / count;
    uint32_t divided_mv;

    divided_mv = (((sample + calc->offset) * calc->mvref) / (1 << 12));
    *value = (divided_mv * (calc->r1 + calc->r2)) / calc->r2;
}

#endif  /* __AT91SAM4S__ */

