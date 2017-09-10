/*
 * saml_tcc.h
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


#ifndef __SAM_TCC_H__
#define __SAM_TCC_H__

typedef struct tcc
{
    uint32_t ctrla;
#define TCC_CTRLA_SWRST                          (1 << 0)
#define TCC_CTRLA_ENABLE                         (1 << 1)
#define TCC_CTRLA_RESOLUTION(val)                ((val & 0x3) << 5)
#define TCC_CTRLA_PRESCALER(val)                 ((val & 0x7) << 8)
#define TCC_CTRLA_PRESCALER_DIV1                 0x0
#define TCC_CTRLA_PRESCALER_DIV2                 0x1
#define TCC_CTRLA_PRESCALER_DIV4                 0x2
#define TCC_CTRLA_PRESCALER_DIV8                 0x3
#define TCC_CTRLA_PRESCALER_DIV16                0x4
#define TCC_CTRLA_PRESCALER_DIV64                0x5
#define TCC_CTRLA_PRESCALER_DIV256               0x6
#define TCC_CTRLA_PRESCALER_DIV1024              0x7
#define TCC_CTRLA_RUNSTDBY                       (1 << 11)
#define TCC_CTRLA_PRESYNC_GCLK                   (0x0 << 12)
#define TCC_CTRLA_PRESYNC_PRESC                  (0x1 << 12)
#define TCC_CTRLA_PRESYNC_RESYNC                 (0x2 << 12)
#define TCC_CTRLA_ALOCK                          (1 << 14)
#define TCC_CTRLA_MSYNC                          (1 << 15)
#define TCC_CTRLA_DMAOS                          (1 << 23)
    uint8_t ctrlbclr;
#define TCC_CTRLBCLR_DIR                         (1 << 0)
#define TCC_CTRLBCLR_LUPD                        (1 << 1)
#define TCC_CTRLBCLR_ONESHOT                     (1 << 2)
#define TCC_CTRLBCLR_IDXCMD(val)                 ((val 0x3) << 3)
#define TCC_CTRLBCLR_CMD(val)                    ((val & 0x7) << 5)
    uint8_t ctrlbset;
#define TCC_CTRLBSET_DIR                         (1 << 0)
#define TCC_CTRLBSET_LUPD                        (1 << 1)
#define TCC_CTRLBSET_ONESHOT                     (1 << 2)
#define TCC_CTRLBSET_IDXCMD(val)                 ((val 0x3) << 3)
#define TCC_CTRLBSET_CMD_NONE                    (0 << 5)
#define TCC_CTRLBSET_CMD_RETRIGGER               (1 << 5)
#define TCC_CTRLBSET_CMD_STOP                    (2 << 5)
#define TCC_CTRLBSET_CMD_UPDATE                  (3 << 5)
#define TCC_CTRLBSET_CMD_READSYNC                (4 << 5)
    uint16_t resvd_0x6;
    uint32_t syncbusy;
#define TCC_SYNCBUSY_SWRST                       (1 << 0)
#define TCC_SYNCBUSY_ENABLE                      (1 << 1)
#define TCC_SYNCBUSY_CTRLB                       (1 << 2)
#define TCC_SYNCBUSY_STATUS                      (1 << 3)
#define TCC_SYNCBUSY_COUNT                       (1 << 4)
#define TCC_SYNCBUSY_PATT                        (1 << 5)
#define TCC_SYNCBUSY_WAVE                        (1 << 6)
#define TCC_SYNCBUSY_PER                         (1 << 7)
    uint32_t fctrla;
#define TCC_FCTRLA_SRC(va)                       ((val & 0x3) << 0)
#define TCC_FCTRLA_KEEP                          (1 << 3)
#define TCC_FCTRLA_QUAL                          (1 << 4)
#define TCC_FCTRLA_BLANK(val)                    ((val & 0x3) << 5)
#define TCC_FCTRLA_RESTART                       (1 << 7)
#define TCC_FCTRLA_HALT(val)                     ((val & 0x3) << 8)
#define TCC_FCTRLA_CHSEL(val)                    ((val & 0x3) << 10)
#define TCC_FCTRLA_CAPTURE(val)                  ((val & 0x7) << 12)
#define TCC_FCTRLA_BLANKPREC                     (1 << 15)
#define TCC_FCTRLA_BLANKVAL(val)                 ((val & 0xff) << 16)
#define TCC_FCTRLA_FILTERVAL(val)                ((val & 0xf) << 24)
    uint32_t fctrlb;
#define TCC_FCTRLB_SRC(va)                       ((val & 0x3) << 0)
#define TCC_FCTRLB_KEEP                          (1 << 3)
#define TCC_FCTRLB_QUAL                          (1 << 4)
#define TCC_FCTRLB_BLANK(val)                    ((val & 0x3) << 5)
#define TCC_FCTRLB_RESTART                       (1 << 7)
#define TCC_FCTRLB_HALT(val)                     ((val & 0x3) << 8)
#define TCC_FCTRLB_CHSEL(val)                    ((val & 0x3) << 10)
#define TCC_FCTRLB_CAPTURE(val)                  ((val & 0x7) << 12)
#define TCC_FCTRLB_BLANKPREC                     (1 << 15)
#define TCC_FCTRLB_BLANKVAL(val)                 ((val & 0xff) << 16)
#define TCC_FCTRLB_FILTERVAL(val)                ((val & 0xf) << 24)
    uint32_t wexctrl;
#define TCC_WEXCTRL_OTMX(val)                    ((val & 0x3) << 0)
#define TCC_WEXCTRL_DTIEN(x)                     ((1 << x) << 8)
#define TCC_WEXCTRL_DTLS(val)                    ((val & 0xff) << 16)
#define TCC_WEXCTRL_DTHS(val)                    ((val & 0xff) << 24)
    uint32_t drvctrl;
#define TCC_DRVCTRL_NRE(x)                       ((1 << x) << 0)
#define TCC_DRVCTRL_NRV(x)                       ((1 << x) << 8)
#define TCC_DRVCTRL_INVMASK(x)                   (x << 16)
#define TCC_DRVCTRL_INVEN(x)                     ((1 << x) << 16)
#define TCC_DRVCTRL_FILTERVAL0(val)              ((val & 0xf) << 24)
#define TCC_DRVCTRL_FILTERVAL1(val)              ((val & 0xf) << 28)
    uint16_t resvd_0x1c;
    uint8_t  dbgctrl;
#define TCC_DBGCTRL_DBGRUN                       (1 << 0)
#define TCC_DBGCTRL_FDDBD                        (1 << 3)
    uint8_t  resvd_0x1f;
    uint32_t evctrl;
#define TCC_EVCTRL_EVACT0(val)                   ((val & 0x7) << 0)
#define TCC_EVCTRL_EVACT1(val)                   ((val & 0x7) << 3)
#define TCC_EVCTRL_CNTSEL(val)                   ((val & 0x3) << 6)
#define TCC_EVCTRL_OVFEO                         (1 << 8)
#define TCC_EVCTRL_TRGEO                         (1 << 9)
#define TCC_EVCTRL_CNTEO                         (1 << 10)
#define TCC_EVCTRL_TCINV(x)                      ((1 << x) << 12)
#define TCC_EVCTRL_TCEI(x)                       ((1 << x) << 14)
    uint32_t intenclr;
#define TCC_INTENCLR_OVF                         (1 << 0)
#define TCC_INTENCLR_TRG                         (1 << 1)
#define TCC_INTENCLR_CNT                         (1 << 2)
#define TCC_INTENCLR_ERR                         (1 << 3)
#define TCC_INTENCLR_DFS                         (1 << 11)
#define TCC_INTENCLR_FAULTA                      (1 << 12)
#define TCC_INTENCLR_FAULTB                      (1 << 13)
#define TCC_INTENCLR_FAULT(x)                    ((1 << x) << 14)
    uint32_t intenset;
#define TCC_INTENSET_OVF                         (1 << 0)
#define TCC_INTENSET_TRG                         (1 << 1)
#define TCC_INTENSET_CNT                         (1 << 2)
#define TCC_INTENSET_ERR                         (1 << 3)
#define TCC_INTENSET_DFS                         (1 << 11)
#define TCC_INTENSET_FAULTA                      (1 << 12)
#define TCC_INTENSET_FAULTB                      (1 << 13)
#define TCC_INTENSET_FAULT(x)                    ((1 << x) << 14)
    uint32_t intflag;
#define TCC_INTFLAG_OVF                          (1 << 0)
#define TCC_INTFLAG_TRG                          (1 << 1)
#define TCC_INTFLAG_CNT                          (1 << 2)
#define TCC_INTFLAG_ERR                          (1 << 3)
#define TCC_INTFLAG_DFS                          (1 << 11)
#define TCC_INTFLAG_FAULTA                       (1 << 12)
#define TCC_INTFLAG_FAULTB                       (1 << 13)
#define TCC_INTFLAG_FAULT(x)                     ((1 << x) << 14)
    uint32_t status;
#define TCC_STATUS_STOP                          (1 << 0)
#define TCC_STATUS_IDX                           (1 << 1)
#define TCC_STATUS_DFS                           (1 << 3)
#define TCC_STATUS_SLAVE                         (1 << 4)
#define TCC_STATUS_PATTBUFV                      (1 << 5)
#define TCC_STATUS_WAVEBUFV                      (1 << 6)
#define TCC_STATUS_PERBUFV                       (1 << 7)
#define TCC_STATUS_FAULTAIN                      (1 << 8)
#define TCC_STATUS_FAILTBIN                      (1 << 9)
#define TCC_STATUS_FAULT0IN                      (1 << 10)
#define TCC_STATUS_FAULT1IN                      (1 << 11)
#define TCC_STATUS_FAULTA                        (1 << 12)
#define TCC_STATUS_FAULTB                        (1 << 13)
#define TCC_STATUS_FAULT(x)                      ((1 << x) << 14)
    uint32_t count;
    uint16_t patt;
#define TCC_PATTBUF_PGE0(val)                    ((val & 0xff) << 0)
#define TCC_PATTBUF_PGV0(val)                    ((val & 0xff) << 8)
    uint16_t resv_0x3a;
    uint32_t wave;
#define TCC_WAVE_WAVEGEN_NFRQ                    ((0x0 & 0x7) << 0)
#define TCC_WAVE_WAVEGEN_MFRQ                    ((0x1 & 0x7) << 0)
#define TCC_WAVE_WAVEGEN_NPWM                    ((0x2 & 0x7) << 0)
#define TCC_WAVE_WAVEGEN_DSCRITICAL              ((0x4 & 0x7) << 0)
#define TCC_WAVE_WAVEGEN_DSBOTTOM                ((0x5 & 0x7) << 0)
#define TCC_WAVE_WAVEGEN_DSBOTH                  ((0x6 & 0x7) << 0)
#define TCC_WAVE_WAVEGEN_DSTOP                   ((0x7 & 0x7) << 0)
#define TCC_WAVE_RAMP(val)                       ((val & 0x3) << 4)
#define TCC_WAVE_CIPEREN                         (1 << 7)
#define TCC_WAVE_CICCEN0                         (1 << 8)
#define TCC_WAVE_CICCEN1                         (1 << 9)
#define TCC_WAVE_CICCEN2                         (1 << 10)
#define TCC_WAVE_CICCEN3                         (1 << 11)
#define TCC_WAVE_SWAP0                           (1 << 24)
#define TCC_WAVE_SWAP1                           (1 << 25)
#define TCC_WAVE_SWAP2                           (1 << 26)
#define TCC_WAVE_SWAP3                           (1 << 27)
    uint32_t per;
#define TCC_PER_DITHER(val)                      ((val & 0x3f) << 0)
#define TCC_PER_PER(val)                         ((val & 0x3ffff) << 6)
    uint32_t cc[4];
#define TCC_CC_DITHER(val)                       ((val & 0x3f) << 0)
#define TCC_CC_CC(val)                           ((val & 0x3ffff) << 6)
    uint8_t  resvd_0x44[0x10];
    uint16_t pattbuf;
#define TCC_PATTBUF_PGEB0(val)                   ((val & 0xff) << 0)
#define TCC_PATTBUF_PGVB0(val)                   ((val & 0xff) << 8)
    uint16_t resvd_0x66;
    uint32_t wavebuf;
#define TCC_WAVEBUF_WAVEGENB                     ((val & 0x7) << 0)
#define TCC_WAVEBUF_RAMPB                        ((val & 0x3) << 4)
#define TCC_WAVEBUF_CIPERENB                     (1 << 7)
#define TCC_WAVEBUF_CICCENB0                     (1 << 8)
#define TCC_WAVEBUF_CICCENB1                     (1 << 9)
#define TCC_WAVEBUF_CICCENB2                     (1 << 10)
#define TCC_WAVEBUF_CICCENB3                     (1 << 11)
#define TCC_WAVEBUF_SWAPB0                       (1 << 24)
#define TCC_WAVEBUF_SWAPB1                       (1 << 25)
#define TCC_WAVEBUF_SWAPB2                       (1 << 26)
#define TCC_WAVEBUF_SWAPB3                       (1 << 27)
    uint32_t perbuf;
#define TCC_PERBUF_DITHERBUF(val)                ((val & 0x3f) << 0)
#define TCC_PERBUF_PERBUF(val)                   ((val & 0x3ffff) << 6)
    uint32_t ccbuf[4];
#define TCC_CCBUF_DITHER(val)                    ((val & 0x3f) << 0)
#define TCC_CCBUF_CC(val)                        ((val & 0x3ffff) << 6)
} __attribute__((packed)) tcc_t;


#if defined(__ATSAMD53__)
#define TCC0                                     ((volatile tcc_t *)0x41016000)
#define TCC1                                     ((volatile tcc_t *)0x41018000)
#define TCC2                                     ((volatile tcc_t *)0x42000c00)
#define TCC3                                     ((volatile tcc_t *)0x42001000)
#define TCC4                                     ((volatile tcc_t *)0x43001000)
#endif /* __ATSAMD53__ */


void tcc_pwm_init(volatile tcc_t *tcc, uint32_t prescaler,
                  uint8_t invertmask, uint16_t period);
void tcc_disable(volatile tcc_t *tcc);
void tcc_pwm_duty(volatile tcc_t *tcc, int channel, uint16_t duty);


#endif /* __SAM_TCC_H__ */
