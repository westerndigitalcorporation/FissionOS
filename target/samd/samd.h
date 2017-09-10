/*
 * samd.h
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


#ifndef __SAMD_H__
#define __SAMD_H__


#define GCLK0_HZ                                 47972352
#define GCLK0                                    0
#define GCLK1                                    1

#define DBG_UART_BAUDRATE                        38400

#define CLK_CORE                                 0
#define DBG_UART_ID                              5
#define DBG_UART_GCLK_CORE                       GCLK_SERCOM5_CORE
#define DBG_UART_PERIPHERAL_ID                   PERIPHERAL_ID_SERCOM5
#define DBG_UART_PORT                            PORTB
#define DBG_UART_PORT_TX_PIN                     16
#define DBG_UART_PORT_TX_MUX                     2
#define DBG_UART_TX_PAD                          SERCOM_USART_CTRLA_TXPO_PAD0
#define DBG_UART_PORT_RX_PIN                     17
#define DBG_UART_PORT_RX_MUX                     2
#define DBG_UART_RX_PAD                          SERCOM_USART_CTRLA_RXPO_PAD1

#define BOOTLOADER_SIZE                          (16 * 1024)
#define FLASH_SIZE_BYTES                         (256 * 1024)

#define BOOTCFG_ADDR                             (FLASH_SIZE_BYTES - (8 * 1024))
#define BOOTCFG_MAGIC                            0x90187340


#define SRAM_ADDR                                0x20000000


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


#endif /* __SAMD_H__ */
