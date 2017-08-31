/*
 * sam4_pwm.h
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


#ifndef __SAM4_PWM_H__
#define __SAM4_PWM_H__

typedef struct pwm_cmp
{
    uint32_t cmpv;
    uint32_t cmpvupd;
    uint32_t cmpm;
    uint32_t cmpmupd;
} __attribute__ ((packed)) pwm_cmp_t;

typedef struct pwm_chn
{
    uint32_t cmr;
#define PWM_CMR_CPRE(val)              ((val & 0x0f) << 0)
#define PWM_CMR_CPRE_DIV_1             0
#define PWM_CMR_CPRE_DIV_2             1
#define PWM_CMR_CPRE_DIV_4             2
#define PWM_CMR_CPRE_DIV_8             3
#define PWM_CMR_CPRE_DIV_16            4
#define PWM_CMR_CPRE_DIV_32            5
#define PWM_CMR_CPRE_DIV_64            6
#define PWM_CMR_CPRE_DIV_128           7
#define PWM_CMR_CPRE_DIV_256           8
#define PWM_CMR_CPRE_DIV_512           9
#define PWM_CMR_CPRE_DIV_1024          10
#define PWM_CMR_CPRE_CLKA              11
#define PWM_CMR_CPRE_CLKB              12
#define PWM_CMR_CALG                   (1 << 8)
#define PWM_CMR_CPOL                   (1 << 9)
#define PWM_CMR_CES                    (1 << 10)
#define PWM_CMR_DTE                    (1 << 16)
#define PWM_CMR_DTHI                   (1 << 17)
#define PWM_CMR_DTLI                   (1 << 18)
    uint32_t cdty;
    uint32_t cdtyupd;
    uint32_t cprd;
    uint32_t cprdupd;
    uint32_t ccnt;
    uint32_t dt;
    uint32_t dtupd;
} __attribute__ ((packed)) pwm_chn_t;

typedef struct pwm
{
    uint32_t clk;
#define PWM_CLK_DIVA(val)              ((val & 0xff) << 0)
#define PWM_CLK_PREA(val)              ((val & 0x0f) << 8)
#define PWM_CLK_DIVB(val)              ((val & 0xff) << 16)
#define PWM_CLK_PREB(val)              ((val & 0x0f) << 24)
    uint32_t ena;
    uint32_t dis;
    uint32_t sr;
    uint32_t ier1;
    uint32_t idr1;
    uint32_t imr1;
    uint32_t isr1;
    uint32_t scm;
    uint32_t resvd_0x24;
    uint32_t scuc;
    uint32_t scup;
    uint32_t scupupd;
    uint32_t ier2;
    uint32_t idr2;
    uint32_t imr2;
    uint32_t isr2;
    uint32_t oov;
    uint32_t os;
#define PWM_OS_OSH(val)                ((val & 0x0f) << 0)
#define PWM_OS_OSL(val)                ((val & 0x0f) << 16)
    uint32_t oss;
    uint32_t osc;
    uint32_t ossupd;
    uint32_t oscupd;
    uint32_t fmr;
    uint32_t fsr;
    uint32_t fcr;
    uint32_t fpv;
    uint32_t fpe;
    uint32_t resvd_0x70[3];
    uint32_t elmr0;
    uint32_t elmr1;
    uint32_t resvd_0x84[11];
    uint32_t smmr;
    uint32_t resvd_0xb4[12];
    uint32_t wpcr;
#define PWM_WPCR_WPCMD(val)            ((val & 0x3) << 0)
#define PWM_WPCR_WPRG0                 (1 << 2)
#define PWM_WPCR_WPRG1                 (1 << 3)
#define PWM_WPCR_WPRG2                 (1 << 4)
#define PWM_WPCR_WPRG3                 (1 << 5)
#define PWM_WPCR_WPRG4                 (1 << 6)
#define PWM_WPCR_WPRG5                 (1 << 7)
#define PWM_WPCR_WPKEY                 (0x50574d << 8)
    uint32_t wpsr;
    uint32_t resvd_0xec[17];
    pwm_cmp_t cmp[8];
    uint32_t resvd_0x1b0[20];
    pwm_chn_t chn[4];
} __attribute__ ((packed)) pwm_t;

#if defined(__AT91SAM4S__)
#define PWM                            ((volatile pwm_t *)0x40020000)
#elif defined(__AT91SAM4E__)
#define PWM                            ((volatile pwm_t *)0x40000000)
#endif

typedef struct pwm_drv
{
    uint32_t channel_num;
    volatile gpio_regs_t *gpio;
    uint32_t pin;
    uint32_t pin_peripheral;
    uint32_t mck_div;
    uint32_t period;
    uint32_t duty_cycle;
    uint32_t flags;
#define PWM_DRV_START_HIGH              (1 << 0)
} pwm_drv_t;

void pwm_init(pwm_drv_t *pwm, uint32_t count);
void pwm_duty_cycle(pwm_drv_t *pwm, uint32_t duty_cycle);
void pwm_enable(pwm_drv_t *pwm);
void pwm_disable(pwm_drv_t *pwm);


#endif /* __SAM4_PWM_H__ */
