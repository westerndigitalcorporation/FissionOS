/*
 * saml_vectors.h
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


#ifndef __SAML_VECTORS__
#define __SAML_VECTORS__

#ifdef __AT91SAML21__

#define PERIPHERAL_ID_WDT        1
#define PERIPHERAL_ID_RTC        2
#define PERIPHERAL_ID_EIC        3
#define PERPIHERAL_ID_NVMCTRL    4
#define PERIPHERAL_ID_USB        6
#define PERIPHERAL_ID_EVSYS      7
#define PERIPHERAL_ID_SERCOM0    8
#define PERIPHERAL_ID_SERCOM1    9
#define PERIPHERAL_ID_SERCOM2    10
#define PERIPHERAL_ID_SERCOM3    11
#define PERIPHERAL_ID_SERCOM4    12
#define PERIPHERAL_ID_SERCOM5    13
#define PERIPHERAL_ID_TCC0       14
#define PERIPHERAL_ID_TCC1       15
#define PERIPHERAL_ID_TCC2       16
#define PERIPHERAL_ID_TC0        17
#define PERIPHERAL_ID_TC1        18
#define PERIPHERAL_ID_TC2        19
#define PERIPHERAL_ID_TC3        20
#define PERIPHERAL_ID_TC4        21
#define PERIPHERAL_ID_ADC        22
#define PERIPHERAL_ID_AC         23
#define PERIPHERAL_ID_DAC        24
#define PERIPHERAL_ID_PTC        25
#define PERIPHERAL_ID_AES        26
#define PERIPHERAL_ID_TRNG       27

#define ISR_MAX                  28

#endif /* __AT91SAML21__ */

#ifdef __AT91SAMD20__

#define PERIPHERAL_ID_PM         0
#define PERIPHERAL_ID_SYSCTRL    1
#define PERIPHERAL_ID_WDT        2
#define PERIPHERAL_ID_RTC        3
#define PERIPHERAL_ID_EIC        4
#define PERIPHERAL_ID_NVMCTRL    5
#define PERIPHERAL_ID_EVSYS      6
#define PERIPHERAL_ID_SERCOM0    7
#define PERIPHERAL_ID_SERCOM1    8
#define PERIPHERAL_ID_SERCOM2    9
#define PERIPHERAL_ID_SERCOM3    10
#define PERIPHERAL_ID_SERCOM4    11
#define PERIPHERAL_ID_SERCOM5    12
#define PERIPHERAL_ID_TC0        13
#define PERIPHERAL_ID_TC1        14
#define PERIPHERAL_ID_TC2        15
#define PERIPHERAL_ID_TC3        16
#define PERIPHERAL_ID_TC4        17
#define PERIPHERAL_ID_TC5        18
#define PERIPHERAL_ID_TC6        19
#define PERIPHERAL_ID_TC7        20
#define PERIPHERAL_ID_ADC        21
#define PERIPHERAL_ID_AC         22
#define PERIPHERAL_ID_DAC        23
#define PERIPHERAL_ID_PTC        24

#define ISR_MAX                  25

#endif /* __AT91SAMD20__ */

#ifdef __ATSAMD53__

#define PERIPHERAL_ID_PM          0
#define PERIPHERAL_ID_MCLK        1
#define PERIPHERAL_ID_OSCCTRL     2
#define PERIPHERAL_ID_XOSC0       3
#define PERIPHERAL_ID_XOSC1       4
#define PERIPHERAL_ID_DFLL        5
#define PERIPHERAL_ID_DPLL        6
#define PERIPHERAL_ID_OSC32K      7
#define PERIPHERAL_ID_SUPC        8
#define PERIPHERAL_ID_BOD         9
#define PERIPHERAL_ID_WDT         10
#define PERIPHERAL_ID_RTC         11
#define PERIPHERAL_ID_EXTINT0     12
#define PERIPHERAL_ID_EXTINT1     13
#define PERIPHERAL_ID_EXTINT2     14
#define PERIPHERAL_ID_EXTINT3     15
#define PERIPHERAL_ID_EXTINT4     16
#define PERIPHERAL_ID_EXTINT5     17
#define PERIPHERAL_ID_EXTINT6     18
#define PERIPHERAL_ID_EXTINT7     19
#define PERIPHERAL_ID_EXTINT8     20
#define PERIPHERAL_ID_EXTINT9     21
#define PERIPHERAL_ID_EXTINT10    22
#define PERIPHERAL_ID_EXTINT11    23
#define PERIPHERAL_ID_EXTINT12    24
#define PERIPHERAL_ID_EXTINT13    25
#define PERIPHERAL_ID_EXTINT14    26
#define PERIPHERAL_ID_EXTINT15    27
#define PERIPHERAL_ID_FREQM       28
#define PERIPHERAL_ID_NVMCTRL_A   29
#define PERIPHERAL_ID_NVMCTRL_B   30
#define PERIPHERAL_ID_DMAC0       31
#define PERIPHERAL_ID_DMAC1       32
#define PERIPHERAL_ID_DMAC2       33
#define PERIPHERAL_ID_DMAC3       34
#define PERIPHERAL_ID_DMAC4       35
#define PERIPHERAL_ID_EVSYS0      36
#define PERIPHERAL_ID_EVSYS1      37
#define PERIPHERAL_ID_EVSYS2      38
#define PERIPHERAL_ID_EVSYS3      39
#define PERIPHERAL_ID_EVSYS4      40
#define PERIPHERAL_ID_PAC         41
#define PERIPHERAL_ID_RAM_ECC     45
#define PERIPHERAL_ID_SERCOM0     46
#define PERIPHERAL_ID_SERCOM1     50
#define PERIPHERAL_ID_SERCOM2     54
#define PERIPHERAL_ID_SERCOM3     58
#define PERIPHERAL_ID_SERCOM4     62
#define PERIPHERAL_ID_SERCOM5     66
#define PERIPHERAL_ID_SERCOM6     70
#define PERIPHERAL_ID_SERCOM7     74
#define PERIPHERAL_ID_CAN0        78
#define PERIPHERAL_ID_CAN1        79
#define PERIPHERAL_ID_USB         80
#define PERIPHERAL_ID_USB_SOF     81
#define PERIPHERAL_ID_USB_TRCPT0  82
#define PERIPHERAL_ID_USB_TRCPT1  83
#define PERIPHERAL_ID_GMAC        84
#define PERIPHERAL_ID_TCC0        85
#define PERIPHERAL_ID_TCC0_MC0    86
#define PERIPHERAL_ID_TCC0_MC1    87
#define PERIPHERAL_ID_TCC0_MC2    88
#define PERIPHERAL_ID_TCC0_MC3    89
#define PERIPHERAL_ID_TCC0_MC4    90
#define PERIPHERAL_ID_TCC0_MC5    91
#define PERIPHERAL_ID_TCC1        92
#define PERIPHERAL_ID_TCC1_MC0    93
#define PERIPHERAL_ID_TCC1_MC1    94
#define PERIPHERAL_ID_TCC1_MC2    95
#define PERIPHERAL_ID_TCC1_MC3    96
#define PERIPHERAL_ID_TCC2        97
#define PERIPHERAL_ID_TCC2_MC0    98
#define PERIPHERAL_ID_TCC2_MC1    99
#define PERIPHERAL_ID_TCC2_MC2    100
#define PERIPHERAL_ID_TCC3        101
#define PERIPHERAL_ID_TCC3_MC0    102
#define PERIPHERAL_ID_TCC3_MC1    103
#define PERIPHERAL_ID_TCC4        104
#define PERIPHERAL_ID_TCC4_MC0    105
#define PERIPHERAL_ID_TCC4_MC1    106
#define PERIPHERAL_ID_TC0         107
#define PERIPHERAL_ID_TC1         108
#define PERIPHERAL_ID_TC2         109
#define PERIPHERAL_ID_TC3         110
#define PERIPHERAL_ID_TC4         111
#define PERIPHERAL_ID_TC5         112
#define PERIPHERAL_ID_TC6         113
#define PERIPHERAL_ID_TC7         114
#define PERIPHERAL_ID_PDEC        115
#define PERIPHERAL_ID_PDEC_MC0    116
#define PERIPHERAL_ID_PDEC_MC1    117
#define PERIPHERAL_ID_ADC0        118
#define PERIPHERAL_ID_ADC0_RESRDY 119
#define PERIPHERAL_ID_ADC1        120
#define PERIPHERAL_ID_ADC1_RESRDY 121
#define PERIPHERAL_ID_AC          122
#define PERIPHERAL_ID_DAC         123
#define PERIPHERAL_ID_DAC_EMPTY0  124
#define PERIPHERAL_ID_DAC_EMPTY1  125
#define PERIPHERAL_ID_DAC_RDY0    126
#define PERIPHERAL_ID_DAC_RDY1    127
#define PERIPHERAL_ID_I2S         128
#define PERIPHERAL_ID_PCC         129
#define PERIPHERAL_ID_AES         130
#define PERIPHERAL_ID_TRNG        131
#define PERIPHERAL_ID_ICM         132
#define PERIPHERAL_ID_PUKCC       133
#define PERIPHERAL_ID_QSPI        134
#define PERIPHERAL_ID_SDHC0       135
#define PERIPHERAL_ID_SDHC1       136
// TODO:  Finish peripheral IDs
//

#define ISR_MAX                   137

#define PERIPHERAL_ID_SERCOM_INTS 4

#endif /* __ATSAMD53__ */

#endif /* __SAML_VECTORS__ */
