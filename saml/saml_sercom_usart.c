/*
 * saml_sercom_usart.c
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

#include <vectors.h>

#include "saml_sercom.h"


// Don't make a array of these.  If each has its own symbol, they can
// be pruned appropriately by the linker to save memory.
static uart_drv_t *uart_drv[SERCOM_COUNT];

#ifdef __AT91SAML21__
void syncbusy_wait(sercom_usart_t *usart)
{
    while (usart->syncbusy)
        ;
}
#endif /* __AT91SAML21__ */
#ifdef __AT91SAMD20__
void syncbusy_wait(sercom_usart_t *usart)
{
    while (usart->status & SERCOM_USART_STATUS_SYNCBUSY)
        ;
}
#endif /* __AT91SAMD20__ */

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

static void sercom_usart_int_handler(uart_drv_t *uart)
{
    if (uart->dev->intflag & SERCOM_USART_INTFLAG_RXC)
    {
        uart_rxbuf_append(uart, uart->dev->data);
    }

    if (uart->rx_cb)
    {
        uart->rx_cb(uart, uart->rx_arg);
    }
}


// Interrupt handler callbacks
static void sercom_usart0_int_handler(void)
{
    sercom_usart_int_handler(uart_drv[0]);
}

static void sercom_usart1_int_handler(void)
{
    sercom_usart_int_handler(uart_drv[1]);
}

static void sercom_usart2_int_handler(void)
{
    sercom_usart_int_handler(uart_drv[2]);
}

static void sercom_usart3_int_handler(void)
{
    sercom_usart_int_handler(uart_drv[3]);
}

static void sercom_usart4_int_handler(void)
{
    sercom_usart_int_handler(uart_drv[4]);
}

static void sercom_usart5_int_handler(void)
{
    sercom_usart_int_handler(uart_drv[5]);
}


void sercom_usart_async_init(uart_drv_t *uart, uint8_t peripheral_id,
                             uint32_t clockrate, uint32_t baudrate,
                             uint32_t chsize, uint32_t sbmode,
                             uint32_t form, uint32_t pmode,
                             uint32_t txpo, uint32_t rxpo)
{
    uint32_t ctrla = SERCOM_USART_CTRLA_MODE_INTCLK |
                     SERCOM_USART_CTRLA_IBON |
                     SERCOM_USART_CTRLA_RUNSTDBY |
                     SERCOM_USART_CTRLA_DORD_LSB |
                     SERCOM_USART_CTRLA_CMODE_ASYNC |
                     txpo | rxpo | form;
    volatile sercom_usart_t *usart;
    void (*vector)(void);

    switch (peripheral_id)
    {
        case PERIPHERAL_ID_SERCOM0:
            usart = SERCOM0_USART;
            vector = sercom_usart0_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM1:
            usart = SERCOM1_USART;
            vector = sercom_usart1_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM2:
            usart = SERCOM2_USART;
            vector = sercom_usart2_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM3:
            usart = SERCOM3_USART;
            vector = sercom_usart3_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM4:
            usart = SERCOM4_USART;
            vector = sercom_usart4_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM5:
            usart = SERCOM5_USART;
            vector = sercom_usart5_int_handler;
            break;

        default:
            return;
    }
    uart_drv[peripheral_id - PERIPHERAL_ID_SERCOM0] = uart;

    uart->dev = usart;

    usart->ctrla = SERCOM_USART_CTRLA_SWRST;
    syncbusy_wait(usart);

    usart->ctrla = ctrla;
    syncbusy_wait(usart);

    // The baudrate must be less than than clockrate / 16 for the following computation,
    // and the asynchronous arithmetic method to work properly.
    usart->baud = 65536 - (((baudrate / 100) * 16 * 65536) / (clockrate / 100));
    usart->intenset |= SERCOM_USART_INTENSET_RXC;
    usart->ctrlb = SERCOM_USART_CTRLB_TXEN |
                   SERCOM_USART_CTRLB_RXEN |
                   chsize | sbmode | pmode;
    syncbusy_wait(usart);

    // Enable usart
    usart->ctrla |= SERCOM_USART_CTRLA_ENABLE;
    syncbusy_wait(usart);

    nvic_callback_set(peripheral_id, vector);
    nvic_enable(peripheral_id);
}

void sercom_usart_disable(volatile sercom_usart_t *usart)
{
    uint32_t ctrla = usart->ctrla & ~SERCOM_USART_CTRLA_ENABLE;

    usart->ctrla = ctrla;
    syncbusy_wait(usart);
}


void uart_init(uart_drv_t *uart)
{
}

void uart_send_wait(uart_drv_t *uart, char *data, uint32_t len)
{
    while (len)
    {
        uart->dev->data = *data++;
        while (!(uart->dev->intflag & SERCOM_USART_INTFLAG_DRE))
            ;

        len--;
    }
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


