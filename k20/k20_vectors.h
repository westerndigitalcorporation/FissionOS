/*
 * k20_vectors.h
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


#ifndef __K20_VECTORS__
#define __K20_VECTORS__


#define ISR_DMA0                                 0
#define ISR_DMA1                                 1
#define ISR_DMA2                                 2
#define ISR_DMA3                                 3
#define ISR_DMA_ERROR                            4
#define ISR_DMA                                  5
#define ISR_FLASH_CMD_COMPLETE                   6
#define ISR_FLASH_READ_COLLISION                 7
#define ISR_LOW_VOLTAGE                          8
#define ISR_LOW_LEAKAGE_WAKEUP                   9
#define ISR_WATCHDOG                             10
#define ISR_I2C0                                 11
#define ISR_SPI0                                 12
#define ISR_I2SO                                 13
#define ISR_I2S1                                 14
#define ISR_UART0_LON                            15
#define ISR_UART0_STATUS                         16
#define ISR_UART0_ERROR                          17
#define ISR_UART1_STATUS                         18
#define ISR_UART1_ERROR                          19
#define ISR_UART2_STATUS                         20
#define ISR_UART2_ERROR                          21
#define ISR_ADC0                                 22
#define ISR_CMP0                                 23
#define ISR_CMP1                                 24
#define ISR_FTM0                                 25
#define ISR_FTM1                                 26
#define ISR_CMT                                  27
#define ISR_RTC_ALARM                            28
#define ISR_RTC_SECONDS                          29
#define ISR_PIT0                                 30
#define ISR_PIT1                                 31
#define ISR_PIT2                                 32
#define ISR_PIT3                                 33
#define ISR_PDB                                  34
#define ISR_USB_OTG                              35
#define ISR_USB_CHARGER_DETECT                   36
#define ISR_TSI                                  37
#define ISR_MCG                                  38
#define ISR_LOW_POWER_TIMER                      39
#define ISR_PORTA                                40
#define ISR_PORTB                                41
#define ISR_PORTC                                42
#define ISR_PORTD                                43
#define ISR_PORTE                                44
#define ISR_SOFTWARE                             45

#define ISR_MAX                                  46


#endif /* __K20_VECTORS__ */
