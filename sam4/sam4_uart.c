/*
 * sam4_uart.c
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


#include <stdint.h>
#include <stddef.h>

#include "vectors.h"

#include "sam4_gpio.h"
#include "sam4_clock.h"
#include "sam4_uart.h"


#define UART_MAX                  4

#define UART0_INDEX               0
#define UART1_INDEX               1
#define USART0_INDEX              2
#define USART1_INDEX              3

uart_drv_t *uart_drvs[UART_MAX] = { NULL };

static void uart_rxbuf_append(uart_drv_t *uart, uint8_t data)
{
    uint8_t newend = uart->rxbuf_end + 1;

    if (newend >= UART_RX_BUFFER_SIZE)
    {
        newend = 0;
    }

    if (uart->rxbuf_start == newend)
    {
        return;
    }

    uart->rxbuf[uart->rxbuf_end] = data;
    uart->rxbuf_end = newend;
}

static void uart_interrupt_rx(uart_drv_t *uart)
{
    while (uart->dev->csr & UART_CSR_RXRDY)
    {
        uart_rxbuf_append(uart, uart->dev->rhr);
    }

    if (uart->rx_cb)
    {
        uart->rx_cb(uart, uart->rx_arg);
    }
}


/*
 * Interrupt functions
 */
static void uart_interrupt(uart_drv_t *uart)
{
    if (uart->dev->csr & UART_CSR_TXRDY)
    {
        uart->dev->idr = UART_IDR_TXRDY;

        if (uart->tx_cb)
        {
            uart->tx_cb(uart, uart->tx_arg);
        }
    }

    if (uart->dev->csr & UART_CSR_RXRDY)
    {
        uart_interrupt_rx(uart);
    }
}

static void uart0_interrupt(void)
{
    uart_interrupt(uart_drvs[UART0_INDEX]);
}

static void uart1_interrupt(void)
{
    uart_interrupt(uart_drvs[UART1_INDEX]);
}

static void usart0_interrupt(void)
{
    uart_interrupt(uart_drvs[USART0_INDEX]);
}

static void usart1_interrupt(void)
{
    uart_interrupt(uart_drvs[USART1_INDEX]);
}

void uart_interrupt_config(uart_drv_t *uart)
{
    void (*vector)(void) = NULL;
    int id = uart->peripheral_id;

    switch (id)
    {
        case PERIPHERAL_ID_UART0:
            vector = uart0_interrupt;
            uart_drvs[UART0_INDEX] = uart;
            break;

        case PERIPHERAL_ID_UART1:
            vector = uart1_interrupt;
            uart_drvs[UART1_INDEX] = uart;
            break;

        case PERIPHERAL_ID_USART0:
            vector = usart0_interrupt;
            uart_drvs[USART0_INDEX] = uart;
            break;

        case PERIPHERAL_ID_USART1:
            vector = usart1_interrupt;
            uart_drvs[USART1_INDEX] = uart;
            break;

        default:
            break;
    }

    if (vector)
    {
        uart->dev->ier = UART_IER_RXRDY;

        nvic_callback_set(uart->peripheral_id, vector);
        nvic_enable(uart->peripheral_id);
    }
}


/*
 *  Application API Functions
 */

int uart_init(uart_drv_t *uart)
{
    uint32_t parity_mask = 0;

    clock_peripheral_start(uart->peripheral_id);

    // Reset UART state machines
    uart->dev->cr = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RSTSTA;

    // See the Atmel datasheet for baudrate calculation information
    uart->dev->brgr = UART_BRGR_CD(INTERNAL_CLK_FREQ / (16 * uart->baudrate));

    switch (uart->parity)
    {
        case UART_DRV_PARITY_NONE:
            parity_mask = UART_MR_CHRL_PAR_NONE;
            break;

        case UART_DRV_PARITY_EVEN:
            parity_mask = UART_MR_CHRL_PAR_EVEN;
            break;
    }
    uart->dev->mr = parity_mask | UART_MR_8_BIT |
                    UART_MR_CHMODE_NORMAL;

    // Enable UART
    uart->dev->cr = UART_CR_RXEN |  UART_CR_TXEN;

    uart_interrupt_config(uart);

    return 0;
}

void uart_send_wait(uart_drv_t *uart, char *data, uint32_t len)
{
    while (len)
    {
        while (!(uart->dev->csr & UART_CSR_TXRDY))
            ;

        uart->dev->thr = *data++;
        len--;
    }
}

int uart_send(uart_drv_t *uart, char *data, uint32_t len)
{
    uint32_t irqstate = irq_save();
    int sent = 0;

    while ((uart->dev->csr & UART_CSR_TXRDY) && len)
    {
        uart->dev->thr = *data++;
        len--;
        sent++;
    }

    if (uart->tx_cb)
    {
        uart->dev->ier = UART_IER_TXRDY;
    }

    irq_restore(irqstate);

    return sent;
}

int uart_recv(uart_drv_t *uart, uint8_t *data, int maxlen)
{
    uint32_t irqstate = irq_save();
    int len = 0;

    while ((len < maxlen) &&
           (uart->rxbuf_start != uart->rxbuf_end))
    {
        *data++ = uart->rxbuf[uart->rxbuf_start++];
        len++;

        if (uart->rxbuf_start >= UART_RX_BUFFER_SIZE)
        {
            uart->rxbuf_start = 0;
        }
    }

    irq_restore(irqstate);

    return len;
}

static void uart_console_rx_callback(uart_drv_t *uart, void *arg)
{
    console_rx_schedule((console_t *)arg);
}

void uart_console(console_t *console, uart_drv_t *uart)
{
    uart->rx_cb = uart_console_rx_callback;
    uart->rx_arg = console;
}

