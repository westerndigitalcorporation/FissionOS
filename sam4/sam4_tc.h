/*
 * sam4_tc.h
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


#ifndef __SAM4_TC_H__
#define __SAM4_TC_H__


#define SAM4_MAX_CHANNELS                        3

typedef struct tc_channel
{
    uint32_t ccr;           // 0x0
#define TC_CHANNEL_CCR_CLKEN                     (1 << 0)
#define TC_CHANNEL_CCR_CLKDIS                    (1 << 1)
#define TC_CHANNEL_CCR_SWTRG                     (1 << 2)
    uint32_t cmr;           // 0x4
#define TC_CHANNEL_TCCLKS_TIMER_CLOCK1           (0 << 0)   /* Waveform mode */
#define TC_CHANNEL_TCCLKS_TIMER_CLOCK2           (1 << 0)
#define TC_CHANNEL_TCCLKS_TIMER_CLOCK3           (2 << 0)
#define TC_CHANNEL_TCCLKS_TIMER_CLOCK4           (3 << 0)
#define TC_CHANNEL_TCCLKS_TIMER_CLOCK5           (4 << 0)
#define TC_CHANNEL_TCCLKS_XC0                    (5 << 0)
#define TC_CHANNEL_TCCLKS_XC1                    (6 << 0)
#define TC_CHANNEL_TCCLKS_XC2                    (7 << 0)
#define TC_CHANNEL_CLKI                          (1 << 3)
#define TC_CHANNEL_BURST_NONE                    (0 << 4)
#define TC_CHANNEL_BURST_XC0                     (1 << 4)
#define TC_CHANNEL_BURST_XC1                     (2 << 4)
#define TC_CHANNEL_BURST_XC2                     (3 << 4)
#define TC_CHANNEL_LDBSTOP                       (1 << 6)
#define TC_CHANNEL_LDBDIS                        (1 << 7)
#define TC_CHANNEL_ETRGEDG_NONE                  (0 << 8)
#define TC_CHANNEL_ETRGEDG_RISING                (1 << 8)
#define TC_CHANNEL_ETRGEDG_FALLING               (2 << 8)
#define TC_CHANNEL_ETRGEDG_EDGE                  (3 << 8)
#define TC_CHANNEL_ABETRG                        (1 << 10)
#define TC_CHANNEL_CPCTRG                        (1 << 14)
#define TC_CHANNEL_WAVE                          (1 << 15)
#define TC_CHANNEL_LDRA_NONE                     (0 << 16)
#define TC_CHANNEL_LDRA_RISING                   (1 << 16)
#define TC_CHANNEL_LDRA_FALLING                  (2 << 16)
#define TC_CHANNEL_LDRA_EDGE                     (3 << 16)
#define TC_CHANNEL_LDRB_NONE                     (0 << 18)
#define TC_CHANNEL_LDRB_RISING                   (1 << 18)
#define TC_CHANNEL_LDRB_FALLING                  (2 << 18)
#define TC_CHANNEL_LDRB_EDGE                     (3 << 18)
    uint32_t smmr;          // 0x8
    uint32_t resvd_0x0c;    // 0xc
    uint32_t cv;            // 0x10
    uint32_t ra;            // 0x14
    uint32_t rb;            // 0x18
    uint32_t rc;            // 0x1c
    uint32_t sr;            // 0x20
#define TC_CHANNEL_SR_COVFS                      (1 << 0)
#define TC_CHANNEL_SR_LOVRS                      (1 << 1)
#define TC_CHANNEL_SR_CPAS                       (1 << 2)
#define TC_CHANNEL_SR_CPBS                       (1 << 3)
#define TC_CHANNEL_SR_CPCS                       (1 << 4)
#define TC_CHANNEL_SR_LDRAS                      (1 << 5)
#define TC_CHANNEL_SR_LDRBS                      (1 << 6)
#define TC_CHANNEL_SR_ETRGS                      (1 << 7)
#define TC_CHANNEL_SR_CLKSTA                     (1 << 16)
#define TC_CHANNEL_SR_MTIOA                      (1 << 17)
#define TC_CHANNEL_SR_MTIOB                      (1 << 18)
    uint32_t ier;           // 0x24
#define TC_CHANNEL_IER_COVFS                     (1 << 0)
#define TC_CHANNEL_IER_LOVRS                     (1 << 1)
#define TC_CHANNEL_IER_CPAS                      (1 << 2)
#define TC_CHANNEL_IER_CPBS                      (1 << 3)
#define TC_CHANNEL_IER_CPCS                      (1 << 4)
#define TC_CHANNEL_IER_LDRAS                     (1 << 5)
#define TC_CHANNEL_IER_LDRBS                     (1 << 6)
#define TC_CHANNEL_IER_ETRGS                     (1 << 7)
    uint32_t idr;           // 0x28
#define TC_CHANNEL_IDR_COVFS                     (1 << 0)
#define TC_CHANNEL_IDR_LOVRS                     (1 << 1)
#define TC_CHANNEL_IDR_CPAS                      (1 << 2)
#define TC_CHANNEL_IDR_CPBS                      (1 << 3)
#define TC_CHANNEL_IDR_CPCS                      (1 << 4)
#define TC_CHANNEL_IDR_LDRAS                     (1 << 5)
#define TC_CHANNEL_IDR_LDRBS                     (1 << 6)
#define TC_CHANNEL_IDR_ETRGS                     (1 << 7)
    uint32_t imr;           // 0x2c
#define TC_CHANNEL_IMR_COVFS                     (1 << 0)
#define TC_CHANNEL_IMR_LOVRS                     (1 << 1)
#define TC_CHANNEL_IMR_CPAS                      (1 << 2)
#define TC_CHANNEL_IMR_CPBS                      (1 << 3)
#define TC_CHANNEL_IMR_CPCS                      (1 << 4)
#define TC_CHANNEL_IMR_LDRAS                     (1 << 5)
#define TC_CHANNEL_IMR_LDRBS                     (1 << 6)
#define TC_CHANNEL_IMR_ETRGS                     (1 << 7)
    uint32_t resvd_0x30[4]; // 0x30
} __attribute__ ((packed)) tc_channel_t;

typedef struct tc
{
    volatile tc_channel_t channel[SAM4_MAX_CHANNELS];
    uint32_t bcr;           // 0xc0
#define TC_BCR_SYNC                              (1 << 0)
    uint32_t bmr;           // 0xc4
#define TC_BMR_TC0XC0S_TCLK0                     (0 << 0)
#define TC_BMR_TC0XC0S_TIOA1                     (2 << 0)
#define TC_BMR_TC0XC0S_TIOA2                     (3 << 0)
#define TC_BMR_TC1XC1S_TCLK1                     (0 << 2)
#define TC_BMR_TC1XC1S_TIOA0                     (2 << 2)
#define TC_BMR_TC1XC1S_TIOA2                     (3 << 2)
#define TC_BMR_TC2XC2S_TCLK2                     (0 << 4)
#define TC_BMR_TC2XC2S_TIOA1                     (2 << 4)
#define TC_BMR_TC2XC2S_TIOA2                     (3 << 4)
#define TC_BMR_QDEN                              (1 << 8)
#define TC_BMR_POSEN                             (1 << 9)
#define TC_BMR_SPEEDEN                           (1 << 10)
#define TC_BMR_QDTRANS                           (1 << 11)
#define TC_BMR_EDGPHA                            (1 << 12)
#define TC_BMR_INVA                              (1 << 13)
#define TC_BMR_INVB                              (1 << 14)
#define TC_BMR_INVIDX                            (1 << 15)
#define TC_BMR_SWAP                              (1 << 16)
#define TC_BMR_IDXPHB                            (1 << 17)
#define TC_BMR_FILTER                            (1 << 19)
#define TC_BMR_MAXFILT(val)                      ((val & 0x3f) << 20)
    uint32_t qier;          // 0xc8
#define TC_QIER_IDX                              (1 << 0)
#define TC_QIER_DIRCHG                           (1 << 1)
#define TC_QIER_QERR                             (1 << 2)
    uint32_t qidr;          // 0xcc
#define TC_QIDR_IDX                              (1 << 0)
#define TC_QIDR_DIRCHG                           (1 << 1)
#define TC_QIDR_QERR                             (1 << 2)
    uint32_t qimr;          // 0xd0
#define TC_QIMR_IDX                              (1 << 0)
#define TC_QIMR_DIRCHG                           (1 << 1)
#define TC_QIMR_QERR                             (1 << 2)
    uint32_t qisr;          // 0xd4
#define TC_QISR_IDX                              (1 << 0)
#define TC_QISR_DIRCHG                           (1 << 1)
#define TC_QISR_QERR                             (1 << 2)
    uint32_t fmr;           // 0xd8
#define TC_FMR_ENCF0                             (1 << 0)
#define TC_FMR_ENCF1                             (1 << 1)
    uint32_t resvd_0xdc[3]; // 0xdc
    uint32_t wpmr;          // 0xe4
#define TC_WPMR_WPEN                             (1 << 0)
#define TC_WPMR_KEY                              (0x54494d << 8)
} __attribute__ ((packed)) tc_t;


#if defined(__AT91SAM4S__)
#define TC0_BASE_ADDR                            0x40010000
#define TC0                                      ((volatile tc_t *)TC0_BASE_ADDR)

#define TC1_BASE_ADDR                            0x40014000
#define TC1                                      ((volatile tc_t *)TC1_BASE_ADDR)

#define TC_MAX                                   6
#endif


struct tc_drv;
typedef void (*tc_handler_t)(struct tc_drv *drv, uint32_t freq);

typedef struct tc_drv {
    volatile tc_t *tc;
    int channel;
    tc_handler_t handler;
} tc_drv_t;


int tc_init(tc_drv_t *dev);
int tc_capture_freq(tc_drv_t *dev);

int cmd_tc(uart_drv_t *uart, int argc, char *argv[]);


#endif /* __SAM4_TC_H__ */

