/*
 * samd53.h
 *
 *
 * Copyright (c) 2017 Jeremy Garff
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
 * Author: Jeremy Garff <jer@jers.net>
 *
 */


#ifndef __SAMD53_H__
#define __SAMD53_H__

// For UART debugging
//#define UART_CONSOLE

#define GCLK0_HZ                                 120000000
#define UART_SRC_HZ                              48000000

#define DBG_UART_BAUDRATE                        38400

#define CLK_CORE                                 0
#define LOWSPEED_GCLK                            1
#define CLK48MHZ_GCLK                            2
#define CLK2MHZ_GCLK                             3

#define DBG_UART_ID                              0
#define DBG_UART_GCLK_CORE                       GCLK_SERCOM0_CORE
#define DBG_UART_PERIPHERAL_ID                   PERIPHERAL_ID_SERCOM0
#define DBG_UART_PORT                            PORTA
#define DBG_UART_PORT_TX_PIN                     4
#define DBG_UART_PORT_TX_MUX                     3
#define DBG_UART_TX_PAD                          SERCOM_USART_CTRLA_TXPO_PAD0
#define DBG_UART_PORT_RX_PIN                     5
#define DBG_UART_PORT_RX_MUX                     3
#define DBG_UART_RX_PAD                          SERCOM_USART_CTRLA_RXPO_PAD1

#define USB_DP_PORT                              PORTA
#define USB_DP_PIN                               25
#define USB_DP_MUX                               7
#define USB_DN_PORT                              PORTA
#define USB_DN_PIN                               24
#define USB_DN_MUX                               7

#define PWM0_PORT                                PORTA
#define PWM0_PIN                                 8
#define PWM1_PORT                                PORTA
#define PWM1_PIN                                 9
#define PWM2_PORT                                PORTA
#define PWM2_PIN                                 10
#define PWM3_PORT                                PORTA
#define PWM3_PIN                                 11
#define PWM_MUX                                  5

#define ADC0_AIN0_PORT                           PORTA
#define ADC0_AIN0_PIN                            2
#define ADC0_AIN1_PORT                           PORTA
#define ADC0_AIN1_PIN                            3
#define ADC0_AIN2_PORT                           PORTB
#define ADC0_AIN2_PIN                            8
#define ADC0_AIN3_PORT                           PORTB
#define ADC0_AIN3_PIN                            9
#define ADC_MUX                                  2

#define TCC0_MAX                                 48000


#define SRAM_ADDR                                0x20000000

#define VBUS_PORT                                PORTB
#define VBUS_PIN                                 16
#define VBUS_MUX                                 0
#define VBUS_INTNUM                              0

#define OE_PORT                                  PORTA
#define OE_PIN                                   6
#define OE_MUX                                   0

#define LED_PORT                                 PORTB
#define LED_PIN                                  10
#define LED_MUX                                  6
#define LED_TCC                                  TCC1
#define LED_TCC_CHANNEL                          0
#define STATUS_MAX                               16
#define LED_TCC_MAX                              (1 << STATUS_MAX)

typedef struct imghdr
{
    uint32_t stack_ptr;
    uint32_t start_addr;
} __attribute__ ((packed)) imghdr_t;

#define IMGHDR                                   ((volatile imghdr_t *)BOOTLOADER_SIZE)


typedef struct bootcfg
{
    uint32_t magic;
    uint32_t len;
    uint32_t crc;
    uint32_t resvd_0x0c[13]; // PAD to page size
} bootcfg_t;


#endif /* __SAMD53_H__ */
