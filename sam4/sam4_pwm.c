/*
 * sam4_pwm.c
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


#include <stddef.h>
#include <stdint.h>

#include "vectors.h"
#include "systick.h"

#include "sam4_clock.h"
#include "sam4_gpio.h"
#include "sam4_pwm.h"
#include "sam4_uart.h"

void pwm_init(pwm_drv_t *pwm, uint32_t count)
{
    int i;

    clock_peripheral_start(PERIPHERAL_ID_PWM);

    /*
     * The period is the total number of clocks counted before the counter is
     * reset to zero.  The line resets back to its original state at the end
     * of the period.  The duty cycle is the count between 0 and the period where
     * the line state changes.
     */

    for (i = 0; i < count; i++)
    {
        // Select the clock source, period, and duty cycle
        PWM->chn[pwm[i].channel_num].cmr = PWM_CMR_CPRE(pwm[i].mck_div) |
                                           (pwm[i].flags & PWM_DRV_START_HIGH) ? PWM_CMR_CPOL : 0;
        PWM->chn[pwm[i].channel_num].cprd = pwm[i].period;
        PWM->chn[pwm[i].channel_num].cdty = pwm[i].duty_cycle;
    }
}

void pwm_duty_cycle(pwm_drv_t *pwm, uint32_t duty_cycle)
{
    PWM->chn[pwm->channel_num].cdtyupd = duty_cycle;
    pwm->duty_cycle = duty_cycle;
}

void pwm_enable(pwm_drv_t *pwm)
{
    // Fire it up
    PWM->ena = 1 << pwm->channel_num;

    // Route the pwm to the gpio last, so that the wave isn't incorrect
    // as we set it up.
    GPIO_PERIPHERAL_SET(pwm->gpio, pwm->pin, pwm->pin_peripheral);
    GPIO_DISABLE(pwm->gpio, pwm->pin);
}

void pwm_disable(pwm_drv_t *pwm)
{
    GPIO_ENABLE(pwm->gpio, pwm->pin);
    PWM->dis = 1 << pwm->channel_num;
}

