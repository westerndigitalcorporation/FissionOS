/*
 * sam4_uart.h
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

#ifndef __SAM4S_UART_H__
#define __SAM4S_UART_H__

#include <console.h>

typedef struct uart
{
    uint32_t cr;
#define UART_CR_RSTRX               (1 << 2)
#define UART_CR_RSTTX               (1 << 3)
#define UART_CR_RXEN                (1 << 4)
#define UART_CR_RXDIS               (1 << 5)
#define UART_CR_TXEN                (1 << 6)
#define UART_CR_TXDIS               (1 << 7)
#define UART_CR_RSTSTA              (1 << 8)
    uint32_t mr;
#define UART_MR_8_BIT               ((3 & 0x3) << 6)
#define UART_MR_CHRL_PAR_EVEN       ((0 & 0x7) << 9)
#define UART_MR_CHRL_PAR_ODD        ((1 & 0x7) << 9)
#define UART_MR_CHRL_PAR_NONE       ((4 & 0x7) << 9)
#define UART_MR_CHMODE_NORMAL       ((0 & 0x3) << 14)
#define UART_MR_CHMODE_AUTO_ECHO    ((1 & 0x3) << 14)
#define UART_MR_CHMOD_LOCAL_LOOP    ((2 & 0x3) << 14)
#define UART_MR_CHMOD_REMOTE_LOOP   ((3 & 0x3) << 14)
    uint32_t ier;
#define UART_IER_RXRDY              (1 << 0)
#define UART_IER_TXRDY              (1 << 1)
    uint32_t idr;
#define UART_IDR_RXRDY              (1 << 0)
#define UART_IDR_TXRDY              (1 << 1)
    uint32_t imr;
    uint32_t csr;
#define UART_CSR_RXRDY              (1 << 0)
#define UART_CSR_TXRDY              (1 << 1)
#define UART_CSR_OVRE               (1 << 5)
#define UART_CSR_FRAME              (1 << 6)
#define UART_CSR_PARE               (1 << 7)
#define UART_CSR_TXEMPTY            (1 << 9)
#define UART_CSR_TXBUFFE
#define UART_CSR_RXBUFF             (1 << 12)
    uint32_t rhr;
    uint32_t thr;
    uint32_t brgr;
#define UART_BRGR_CD(val)           ((val & 0xffff) << 0)
} __attribute__ ((packed))  uart_t;


#define UART0                       ((volatile uart_t *)0x400e0600)

#if defined(__AT91SAM4S__)
#define UART1                       ((volatile uart_t *)0x400e0800)
#define USART0                      ((volatile uart_t *)0x40024000)
#define USART1                      ((volatile uart_t *)0x40028000)

#elif defined(__AT91SAM4E__)
#define UART1                       ((volatile uart_t *)0x40060600)
#define USART0                      ((volatile uart_t *)0x400a0000)
#define USART1                      ((volatile uart_t *)0x400a4000)
#endif

#define UART_RX_BUFFER_SIZE         32

struct uart_drv;
typedef void (*uart_callback_t)(struct uart_drv *uart, void *arg);
typedef struct uart_drv
{
    volatile uart_t *dev;
    uint32_t baudrate;
    uint8_t peripheral_id;
    uint8_t parity;
#define UART_DRV_PARITY_NONE        0
#define UART_DRV_PARITY_EVEN        1
#define UART_DRV_PARITY_ODD         2
    uint16_t flags;

    uart_callback_t rx_cb;
    void *rx_arg;
    uint8_t rxbuf[UART_RX_BUFFER_SIZE];
    volatile uint8_t rxbuf_start;
    volatile uint8_t rxbuf_end;

    uart_callback_t tx_cb;
    void *tx_arg;
} uart_drv_t;


int uart_init(uart_drv_t *uart);
void uart_send_wait(uart_drv_t *uart, char *data, uint32_t len);
int uart_send(uart_drv_t *uart, char *data, uint32_t len);
int uart_recv(uart_drv_t *uart, uint8_t *data, int maxlen);
void uart_console(console_t *console, uart_drv_t *uart);

#endif /* __SAM4S_UART_H__ */
