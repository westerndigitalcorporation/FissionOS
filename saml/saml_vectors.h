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

#endif /* __SAML_VECTORS__ */
