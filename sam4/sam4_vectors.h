/*
 * sam4_vectors.h
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


#ifndef __SAM4_VECTORS__
#define __SAM4_VECTORS__

#if defined(__AT91SAM4S__)
#define PERIPHERAL_ID_RTC        2
#define PERIPHERAL_ID_RTT        3
#define PERIPHERAL_ID_WDT        4
#define PERIPHERAL_ID_PMC        5
#define PERIPHERAL_ID_EEFC0      6
#define PERIPHERAL_ID_EEFC1      7
#define PERIPHERAL_ID_UART0      8
#define PERIPHERAL_ID_UART1      9
#define PERIPHERAL_ID_SMC        10
#define PERIPHERAL_ID_PIOA       11
#define PERIPHERAL_ID_PIOB       12
#define PERIPHERAL_ID_PIOC       13
#define PERIPHERAL_ID_USART0     14
#define PERIPHERAL_ID_USART1     15
#define PERIPHERAL_ID_HSMCI      18
#define PERIPHERAL_ID_TWI0       19
#define PERIPHERAL_ID_TWI1       20
#define PERIPHERAL_ID_SPI        21
#define PERIPHERAL_ID_SSC        22
#define PERIPHERAL_ID_TC0        23
#define PERIPHERAL_ID_TC1        24
#define PERIPHERAL_ID_TC2        25
#define PERIPHERAL_ID_TC3        26
#define PERIPHERAL_ID_TC4        27
#define PERIPHERAL_ID_TC5        28
#define PERIPHERAL_ID_ADC        29
#define PERIPHERAL_ID_DACC       30
#define PERIPHERAL_ID_PWM        31
#define PERIPHERAL_ID_CRCCU      32
#define PERIPHERAL_ID_ACC        33
#define PERIPHERAL_ID_UDP        34
#define ISR_MAX                  35

#elif defined(__AT91SAM4E__)
#define PERIPHERAL_ID_RTC        2
#define PERIPHERAL_ID_RTT        3
#define PERIPHERAL_ID_WDT        4
#define PERIPHERAL_ID_PMC        5
#define PERIPHERAL_ID_EEFC0      6
#define PERIPHERAL_ID_UART0      7
#define PERIPHERAL_ID_SMC        8
#define PERIPHERAL_ID_PIOA       9
#define PERIPHERAL_ID_PIOB       10
#define PERIPHERAL_ID_PIOC       11
#define PERIPHERAL_ID_PIOD       12
#define PERIPHERAL_ID_PIOE       13
#define PERIPHERAL_ID_USART0     14
#define PERIPHERAL_ID_USART1     15
#define PERIPHERAL_ID_HSMCI      16
#define PERIPHERAL_ID_TWI0       17
#define PERIPHERAL_ID_TWI1       18
#define PERIPHERAL_ID_SPI        19
#define PERIPHERAL_ID_DMAC       20
#define PERIPHERAL_ID_TC0        21
#define PERIPHERAL_ID_TC1        22
#define PERIPHERAL_ID_TC2        23
#define PERIPHERAL_ID_TC3        24
#define PERIPHERAL_ID_TC4        25
#define PERIPHERAL_ID_TC5        26
#define PERIPHERAL_ID_TC6        27
#define PERIPHERAL_ID_TC7        28
#define PERIPHERAL_ID_TC8        29
#define PERIPHERAL_ID_AFEC0      30
#define PERIPHERAL_ID_AFEC1      31
#define PERIPHERAL_ID_DACC       32
#define PERIPHERAL_ID_ACC        33
#define PERIPHERAL_ID_PWM        36
#define PERIPHERAL_ID_EMAC       44
#define PERIPHERAL_ID_UART1      45
#define ISR_MAX                  46
#endif

#endif /* __SAM4_VECTORS__ */
