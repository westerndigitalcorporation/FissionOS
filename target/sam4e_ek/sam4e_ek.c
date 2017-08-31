/*
 * sam4e_ek.c
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
#include <stddef.h>
#include <stdio.h>

#include "vectors.h"
#include "context.h"
#include "systick.h"

#include "sam4e_ek_version.h"

#include "sam4_gpio.h"
#include "sam4_uart.h"
#include "sam4_watchdog.h"
#include "sam4_clock.h"
#include "sam4_flash.h"
#include "sam4_inputchange.h"
#include "sam4_pwm.h"
#include "sam4_dac.h"
#include "sam4_ac.h"
#include "sam4_reset.h"
#include "sam4_gmac.h"
#include "sam4_uart.h"

#include "console.h"
#include "workqueue.h"
#include "mailbox.h"
#include "semaphore.h"
#include "flash.h"
#include "config.h"

#include "lwip/ip.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/stats.h"
#include "ethernetif.h"

#include "http.h"

#include "swd.h"
#include "swd_driver.h"


#define PHY_ADDR                      0x01


uint8_t mac_addr[6] = { 0xb8, 0x27, 0xeb, 0x05, 0x45, 0xd5 };

/* Network interface global variables */
static struct ip_addr ipaddr, netmask;
static struct ip_addr gw;

/*
 * Command callbacks
 */
int cmd_reset(uart_drv_t *uart, int argc, char *argv[]);
int cmd_ipstats(uart_drv_t *uart, int argc, char *argv[]);
int cmd_threads(uart_drv_t *uart, int argc, char *argv[]);
cmd_entry_t cmd_table[] =
{
    {
        .cmdstr = "ipstats",
        .callback = cmd_ipstats,
    },
    {
        .cmdstr = "eth",
        .callback = cmd_eth,
    },
    {
        .cmdstr = "help",
        .callback = cmd_help,
    },
    {
        .cmdstr = "flash",
        .callback = cmd_flash,
    },
    {
        .cmdstr = "reset",
        .callback = cmd_reset,
    },
};

int cmd_ipstats(uart_drv_t *uart, int argc, char *argv[])
{
    stats_display();
    console_print("\r\n");
    return 0;
}


const http_cgi_table_t http_cgi[] =
{
};
const uint32_t http_cgi_table_count = ARRAY_SIZE(http_cgi);


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
        .gpio = GPIOD,
        .pin = 20,
        .pin_peripheral = GPIO_PERIPHERAL_A,
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
 * Console
 */
#define CONSOLE_GPIO_RXPORT        GPIOA
#define CONSOLE_GPIO_TXPORT        GPIOA
#define CONSOLE_RX_PIN             9
#define CONSOLE_TX_PIN             10
uart_drv_t console_uart =
{
    .dev = UART0,
    .peripheral_id = PERIPHERAL_ID_UART0,
    .baudrate = 115200,
    .parity = UART_DRV_PARITY_NONE,
};


gmac_txbuf_desc_t gmac_txdesc[2];
uint8_t gmac_txpool[GMAC_TXBUF_SIZE * ARRAY_SIZE(gmac_txdesc)];
gmac_rxbuf_desc_t gmac_rxdesc[2];
uint8_t gmac_rxpool[GMAC_RXBUF_SIZE * ARRAY_SIZE(gmac_rxdesc)];
gmac_drv_t gmac =
{
    .txdesc = gmac_txdesc,
    .txbuf_count = ARRAY_SIZE(gmac_txdesc),
    .rxdesc = gmac_rxdesc,
    .rxbuf_count = ARRAY_SIZE(gmac_rxdesc),
};

mailbox_t net_start;
void gmac_link_change_worker(void *arg);
static workqueue_t link_wq =
{
    .callback = gmac_link_change_worker,
};
void gmac_link_change_worker(void *arg)
{
    if (gmac_link_change_handle())
    {
        workqueue_add(&link_wq, SYSTICK_FREQ);
        return;
    }

    mailbox_send(&net_start, NULL);
}

swd_t swd_sam4e;
swd_t *swd_instance = &swd_sam4e;
swd_sam4e_driver_t swd_driver =
{
    // TODO:  Hook up SWD driver pins
};

int main(int argc, char *argv[])
{
    struct netconn *http_netconn;
    void *msg;

    wdt_disable();

    clock_init();
    systick_init(INTERNAL_CLK_FREQ);
    thread_init();

    LED_INIT(GPIOD, (1 << 22));
    LED_ON(GPIOD, (1 << 22));

    swd_sam4e_driver_init(swd_instance, &swd_driver);

    gmac_init(&gmac, mac_addr, PHY_ADDR, gmac_txpool, gmac_rxpool);

    pwm_init(pwms, ARRAY_SIZE(pwms));
    pwm_enable(led_throbber);
    led_handle_work(NULL);

    console_init(&console_uart, cmd_table, ARRAY_SIZE(cmd_table),
                 CONSOLE_GPIO_TXPORT, CONSOLE_TX_PIN,
                 CONSOLE_GPIO_RXPORT, CONSOLE_RX_PIN);

    console_print("\r\nSAM4E-EK Ver %d.%d.%d\r\n",
                  VERSION_MAJOR, VERSION_MINOR, VERSION_MICRO);
    console_prompt();

    // Wait for link up before continuing
    workqueue_add(&link_wq, SYSTICK_FREQ);
    mailbox_recv(&net_start, &msg, 0);

    tcpip_init(NULL, NULL);
    netif_add(&gmac.netif, &ipaddr, &netmask, &gw, NULL, ethernetif_init, tcpip_input);
    netif_set_default(&gmac.netif);
    netif_set_up(&gmac.netif);

	dhcp_start(&gmac.netif);

    http_netconn = http_service_start();
    if (http_netconn)
    {
        http_mainloop(http_netconn);
    }

    return 0;
}

