/*
 * sam4s_xplained.c
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

#include "sam4s_xplained_version.h"

#include "sam4_gpio.h"
#include "sam4_uart.h"
#include "sam4_watchdog.h"
#include "sam4_clock.h"
#include "sam4_flash.h"
#include "sam4_inputchange.h"
#include "sam4_pwm.h"
#include "sam4_ac.h"
#include "sam4_reset.h"
#include "sam4_twi.h"
#include "sam4_uart.h"

#include "vectors.h"
#include "systick.h"

#include "console.h"
#include "workqueue.h"
#include "flash.h"
#include "config.h"


#define BUTTON_PORT                              GPIOA
#define BUTTON_PIN                               5
#define LED0_PORT                                GPIOC
#define LED0_PIN                                 10
#define LED1_PORT                                GPIOC
#define LED1_PIN                                 17


#define MIDPROM_ADDR                             0xa2
twi_drv_t twi =
{
    .twi = TWI0,
};

void cmd_twi_complete(twi_drv_t *t, void *arg, int result)
{
    console_print("Complete: %d\r\n", result);
}

uint8_t twi_buf[2];
int cmd_twi_test(uart_drv_t *uart, int argc, char *argv[])
{
    twi_init(&twi, PERIPHERAL_ID_TWI0);

    twi_master_xfer(&twi, MIDPROM_ADDR, NULL, 0, twi_buf, sizeof(twi_buf), cmd_twi_complete, NULL);

    return 0;
}

/*
 * Command callbacks
 */
int cmd_reset(uart_drv_t *uart, int argc, char *argv[]);
int cmd_pmp(uart_drv_t *uart, int argc, char *argv[]);
cmd_entry_t cmd_table[] =
{
    {
        .cmdstr = "help",
        .callback = cmd_help,
    },
    {
        .cmdstr = "flash",
        .callback = cmd_flash,
    },
    {
        .cmdstr = "config",
        .callback = cmd_config,
    },
    {
        .cmdstr = "reset",
        .callback = cmd_reset,
    },
    {
        .cmdstr = "twi",
        .callback = cmd_twi_test,
    },
};


/*
 * Input Change
 */
void button_change_callback(input_change_t *ic)
{
    if (ic->last_state)
    {
        GPIO_SET_LOW(LED0_PORT, LED0_PIN);
    }
    else
    {
        GPIO_SET_HIGH(LED0_PORT, LED0_PIN);
    }
}

input_change_t buttons[] =
{
    {
        .port = GPIOA,
        .mask = (1 << 5),
        .callback = button_change_callback,
    },
};


/*
 * Console
 */
#define CONSOLE_GPIO_RXPORT        GPIOA
#define CONSOLE_GPIO_TXPORT        GPIOA
#define CONSOLE_RX_PIN             21
#define CONSOLE_TX_PIN             22
uart_drv_t console_uart =
{
    .dev = USART1,
    .peripheral_id = PERIPHERAL_ID_USART1,
    .baudrate = 115200,
    .parity = UART_DRV_PARITY_NONE,
};

void hw_init(void)
{
    GPIO_PULLUP_DISABLE(LED0_PORT, LED0_PIN);
    GPIO_SET_OUTPUT(LED0_PORT, LED0_PIN);
    GPIO_SET_LOW(LED0_PORT, LED0_PIN);
    GPIO_ENABLE(LED0_PORT, LED0_PIN);

    GPIO_PULLUP_DISABLE(LED1_PORT, LED1_PIN);
    GPIO_SET_OUTPUT(LED1_PORT, LED1_PIN);
    GPIO_SET_LOW(LED1_PORT, LED1_PIN);
    GPIO_ENABLE(LED1_PORT, LED1_PIN);

    GPIO_PULLUP_ENABLE(BUTTON_PORT, BUTTON_PIN);
}

int main(int argc, char *argv[])
{
    wdt_disable();

    hw_init();
    clock_init();

    systick_init(INTERNAL_CLK_FREQ);

    clock_peripheral_start(PERIPHERAL_ID_PIOA);
    clock_peripheral_start(PERIPHERAL_ID_PIOB);
    clock_peripheral_start(PERIPHERAL_ID_PIOC);

    console_init(&console_uart, cmd_table, ARRAY_SIZE(cmd_table),
                 CONSOLE_GPIO_TXPORT, CONSOLE_TX_PIN,
                 CONSOLE_GPIO_RXPORT, CONSOLE_RX_PIN);

    // Button pullups
    input_change_init(buttons, ARRAY_SIZE(buttons));

    console_print("\r\nSAM4S-XPLAINED Ver %d.%d.%d\r\n",
                  VERSION_MAJOR, VERSION_MINOR, VERSION_MICRO);
    console_prompt();

    while (1)
    {
        if (!workqueue_handle_next())
        {
            cpu_sleep();
        }
    }

    return 0;
}

