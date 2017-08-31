/*
 * saml21_xplained_pro.c
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
#include <string.h>
#include <stdio.h>

#include "saml_clocks.h"
#include "saml_port.h"
#include "saml_sercom.h"
#include "saml_tc.h"
#include "saml_nvm.h"
#include "saml_reset.h"

#include <vectors.h>
#include <systick.h>
#include <workqueue.h>
#include <console.h>

#include "saml21_xplained_pro_bootloader_version.h"
#include "saml21_xplained_pro_bootloader.h"

#include "saml_xmodem.h"


extern uint32_t __config_word;


//
// TODO:  Remove the following place holder functions when suitable functions
// are available.
//
void thread_switch_handler(void)
{
}

//
// Console Commands
//
uart_drv_t dbg_uart;
cmd_entry_t cmd_table[] =
{
    CONSOLE_CMD_HELP,
    CONSOLE_CMD_NVM,
    CONSOLE_CMD_RESET,
    CONSOLE_CMD_UPLOAD,
};


//
// LED Pulse
//
void led_handle_work(void *arg);
workqueue_t led_wq =
{
    .callback = led_handle_work,
};

#define LED_TICKS_SEC                16
void led_handle_work(void *arg)
{
    static uint32_t duty_cycle = 0;

    duty_cycle++;
    if (duty_cycle >= LED_TICKS_SEC)
    {
        duty_cycle = 0;
    }

    tc_pwm_duty(LED_TC, (1 << duty_cycle) - 1);

    workqueue_add(&led_wq, SYSTICK_FREQ / LED_TICKS_SEC);
}

int bootloader_active(void)
{
    // TODO:  Should test a GPIO for forcing bootloader mode

    if ((RESET->rcause & RESET_RCAUSE_SYST) &&
        (RESET_CONFIG == RESET_CONFIG_BOOTLOADER))
    {
        return 1;
    }

    return 0;
}

void application_start(void)
{
    bootcfg_t *bootcfg = (bootcfg_t *)BOOTCFG_ADDR;
    void (*app)(void) = (void (*)(void))IMGHDR->start_addr;
    uint32_t *dst = (uint32_t *)SRAM_ADDR;
    uint32_t *src = (uint32_t *)BOOTLOADER_SIZE;
    uint32_t len = bootcfg->len / sizeof(uint32_t);
    void *sp = (void *)IMGHDR->stack_ptr;

    if (bootcfg->magic == BOOTCFG_MAGIC)
    {
        // TODO: Validate CRC

        while (len--)
        {
            *dst++ = *src++;
        }

        // Set the stack pointer
        asm volatile ("msr psp, %0;"
                      :                 /* no output */
                      : "r"(sp)         /* stack pointer input */
                      :                 /* no clobbered register */
                );

        app();
    }
}


//
// Main initialization
//
int main(int argc, char *argv[])
{
    if (!bootloader_active())
    {
        application_start();
    }

    systick_init(GCLK0_HZ);

    // Start the pulsing LED
    gclk_peripheral_enable(LED_GCLK, LED_GCLK_PERIPHERAL);
    port_peripheral_enable(LED_PORT, LED_PIN, LED_TC_MUX);
    tc_pwm_init(LED_TC, TC_CTRLA_PRESCALER_DIV1, 1, 0);
    led_handle_work(NULL);

    // Setup the debug UART
    gclk_peripheral_enable(DBG_UART_GCLK, DBG_UART_GCLK_PERIPHERAL);
    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_TX_PIN, DBG_UART_PORT_TX_MUX);
    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_RX_PIN, DBG_UART_PORT_RX_MUX);
    sercom_usart_async_init(&dbg_uart, DBG_UART_PERIPHERAL_ID,
                            GCLK0_HZ, DBG_UART_BAUDRATE,
                            SERCOM_USART_CTRLB_CHSIZE_8BITS, SERCOM_USART_CTRLB_SBMODE_1BIT,
                            SERCOM_USART_CTRLA_FORM_FRAME, SERCOM_USART_CTRLB_PMODE_EVEN,
                            DBG_UART_TX_PAD, DBG_UART_RX_PAD);

    // Setup the console, print version info and prompt
    console_init(&dbg_uart, cmd_table, ARRAY_SIZE(cmd_table));
    console_print("\r\nSAML21-XPLAINED-PRO Bootloader Ver %d.%d.%d\r\n",
                  VERSION_MAJOR, VERSION_MINOR, VERSION_MICRO);
    console_prompt();

    // Mainloop
    while (1)
    {
        if (!workqueue_handle_next())
        {
            cpu_sleep();
        }
    }

    return 0;
}

