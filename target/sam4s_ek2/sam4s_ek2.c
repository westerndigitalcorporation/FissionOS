/*
 * sam4s_ek2.c
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

#include "sam4s_ek2_version.h"

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

#define LED0_PORT                                GPIOC
#define LED0_PIN                                 (1 << 20)
#define LED1_PORT                                GPIOA
#define LED1_PIN                                 (1 << 20)

console_t console;
/*
 * Command callbacks
 */
int cmd_reset(console_t *console, int argc, char *argv[]);
int cmd_twi_test(console_t *console, int argc, char *argv[]);
int cmd_pmp(console_t *console, int argc, char *argv[]);
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
};


/*
 * PWM
 */
void led_handle_work(void *arg);
workqueue_t led_work =
{
    .callback = led_handle_work,
    .arg = NULL,
};

#define LED_PWM_HZ       10000UL
pwm_drv_t pwms[] =
{
    [0] = {
        .channel_num = 0,
        .gpio = GPIOA,
        .pin = 19,
        .pin_peripheral = GPIO_PERIPHERAL_B,
        .mck_div = PWM_CMR_CPRE_DIV_1,
        .period = INTERNAL_CLK_FREQ / LED_PWM_HZ,
        .duty_cycle = 0,
        .flags = PWM_DRV_START_HIGH,
    }
};
pwm_drv_t *led_throbber = &pwms[0];

#define LED_TICKS_SEC                30
void led_handle_work(void *arg)
{
    static uint32_t duty_cycle = 0;
    static uint32_t direction = 1;

    if (direction)
    {
        duty_cycle++;
        if (duty_cycle >= LED_TICKS_SEC)
        {
            direction = 0;
        }
    }
    else
    {
        duty_cycle--;
        if (!duty_cycle)
        {
            direction = 1;
        }
    }

    pwm_duty_cycle(led_throbber, (((INTERNAL_CLK_FREQ / LED_PWM_HZ) * duty_cycle) / LED_TICKS_SEC));

    workqueue_add(&led_work, SYSTICK_FREQ / LED_TICKS_SEC);
}


/*
 * Input Change
 */
void button_change_callback(input_change_t *ic)
{
    if (ic->last_state)
    {
        LED_OFF(LED0_PORT, LED0_PIN);
    }
    else
    {
        LED_ON(LED0_PORT, LED0_PIN);
    }
}

input_change_t buttons[] =
{
    {
        .port = GPIOC,
        .mask = (1 << 12),
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

int main(int argc, char *argv[])
{
    wdt_disable();

    clock_init();
    config_image_switch();
    systick_init(INTERNAL_CLK_FREQ);

    clock_peripheral_start(PERIPHERAL_ID_PIOA);
    clock_peripheral_start(PERIPHERAL_ID_PIOB);
    clock_peripheral_start(PERIPHERAL_ID_PIOC);

    GPIO_DISABLE(CONSOLE_GPIO_TXPORT, CONSOLE_TX_PIN);
    GPIO_DISABLE(CONSOLE_GPIO_RXPORT, CONSOLE_RX_PIN);
    GPIO_PERIPHERAL_SET(CONSOLE_GPIO_TXPORT, CONSOLE_TX_PIN, 0);
    GPIO_PERIPHERAL_SET(CONSOLE_GPIO_RXPORT, CONSOLE_RX_PIN, 0);
    uart_console(&console, &console_uart);

    console_init(&console, cmd_table, ARRAY_SIZE(cmd_table),
                 (console_send_t)uart_send_wait, 
                 (console_recv_t)uart_recv,
                 &console_uart);

    // RED LED
    LED_INIT(LED0_PORT, LED0_PIN);

    // Green LED
    LED_INIT(LED1_PORT, LED1_PIN);
    LED_OFF(LED1_PORT, LED1_PIN);

    // Button pullups
    GPIO_PULLUP_ENABLE(GPIOB, 3);
    GPIO_PULLUP_ENABLE(GPIOC, 12);
    input_change_init(buttons, ARRAY_SIZE(buttons));

    pwm_init(pwms, ARRAY_SIZE(pwms));
    pwm_enable(led_throbber);
    led_handle_work(NULL);

    console_print(&console, "\r\nSAM4S-EK2 Ver %d.%d.%d\r\n",
                  VERSION_MAJOR, VERSION_MINOR, VERSION_MICRO);
    console_print(&console, "Running Bank %d\r\n", eefc_boot_bank());
    console_prompt(&console);

    while (1)
    {
        if (!workqueue_handle_next())
        {
            cpu_sleep();
        }
    }

    return 0;
}

