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

#include "saml_power.h"
#include "saml_clocks.h"
#include "saml_port.h"
#include "saml_sercom.h"
#include "saml_tc.h"
#include "saml_nvm.h"
#include "saml_reset.h"
#include "saml_usb.h"

#include <vectors.h>
#include <systick.h>
#include <workqueue.h>
#include <console.h>
#include <smbus_master.h>
#include <usb.h>

#include "usb_config.h"

#include "saml21_xplained_pro_version.h"
#include "saml21_xplained_pro.h"


console_t console;

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
    CONSOLE_CMD_I2C,
    CONSOLE_CMD_NVM,
    CONSOLE_CMD_RESET,
    CONSOLE_CMD_SMBUS,
    CONSOLE_CMD_USB,
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

    tc_pwm_duty(LED_TC, 0, (1 << duty_cycle) - 1);

    workqueue_add(&led_wq, SYSTICK_FREQ / LED_TICKS_SEC);
}


/*
 * Emulate a simple 256 byte EEPROM
 */
#define EEPROM_SIZE                              256
#define EEPROM_STATE_ADDR                        0
#define EEPROM_STATE_DATA                        1
uint8_t eeprom_data[EEPROM_SIZE] = { 0 };
uint8_t eeprom_offset = 0;
uint8_t eeprom_state = EEPROM_STATE_ADDR;


void eeprom_slave_start(twi_drv_t *twi, void *arg)
{
    eeprom_state = EEPROM_STATE_ADDR;
}

void eeprom_slave_write(twi_drv_t *twi, void *arg, uint8_t data)
{
    switch (eeprom_state)
    {
        case EEPROM_STATE_ADDR:
            eeprom_offset = data;
            eeprom_state = EEPROM_STATE_DATA;
            break;
        case EEPROM_STATE_DATA:
            eeprom_data[eeprom_offset++] = data;
            break;
    }
}

void eeprom_slave_read(twi_drv_t *twi, void *arg)
{
    twi_slave_send(twi, eeprom_data[eeprom_offset++]);
}

// For I2c interface
twi_drv_t twi =
{
    .slave_addr = 0xa0,
    .type = TWI_DRV_TYPE_SLAVE,
    .start = eeprom_slave_start,
    .read = eeprom_slave_read,
    .write = eeprom_slave_write,
};

// External interrupts
void button_callback(void *arg)
{
    // TODO:  Set a workqueue defer and test the pin later to debounce
}

ext_int_t button =
{
    .callback = button_callback,
    .arg = NULL,
};


void power_init(void)
{
    volatile supc_t *supc = SUPC;
    volatile pm_t *pm = PM;
    volatile uint32_t tmp = pm->ctrla;

    supc->vreg |= SUPC_VREG_SEL | SUPC_VREG_VSVSTEP(0) |
                  SUPC_VREG_RUNSTDBY | SUPC_VREG_STDBYPL0 |
                  SUPC_VREG_LPEFF;
    while (!(supc->status & SUPC_STATUS_VREGRDY))
        ;

    tmp = PM_PLCFG_PLSEL(2);
    pm->plcfg = tmp;
    while (!(pm->intflag & PM_INTFLAG_PLRDY))
        ;
}

void clocks_init(void)
{
    volatile nvmctrl_t *nvm = NVMCTRL;
    volatile oscctrl_t *osc = OSCCTRL;
    volatile osc32kctrl_t *osc32 = OSC32KCTRL;

    /*
     * Setup the DFLL for full speed using closed loop mode referenced
     * from a external 32Khz crystal as follows:
     *
     * XOSC32K --> GCLK1 (32Khz) --> DFLL (48Mhz) --> GCLK0
     *
     * As a backup low power option, we'll configure the OSC16M to run
     * at 8Mhz.  Both the DFLL and the OSC16M are configured as ONDEMAND.
     *
     * The GCLK0 source can be switched between the DFLL and OSC16M clock
     * sources.  The switch has a clock startup time penalty, but
     * because only the active source is running, power is reduced.
     */

    // CAUTION: Assuming 3.3 volt operation.
    // Flash must be configured for 1 (PL0) and 2 (PL2) wait state @3v3
    // before going full speed. At 1v8, 3 wait states must be used.
    nvm->ctrlb = NVMCTRL_CTRLB_MANW | NVMCTRL_CTRLB_RWS(2);

    // Turn off the internal 32Khz clock to save power, isn't used.
    osc32->osc32k = 0;

    // Turn on the 32Khz external clock
    osc32->xosc32k = OSC32KCTRL_XOSC32K_ENABLE |
                     OSC32KCTRL_XOSC32K_XTALEN |
                     OSC32KCTRL_XOSC32K_EN32K;
    // Wait for lock
    while (!(osc32->status & OSC32KCTRL_STATUS_XOSC32KRDY))
        ;

    // Set the external 32Khz oscillator as the source of GCLK1
    // and make sure the output of GCLK1 is sent to the DFLL
    gclk_setup(GCLK1, GCLK_GENCTRL_SRC_XOSC32K, 0);
    gclk_peripheral_enable(GCLK1, GCLK_DFLL48M_REF);

    // Setup the DFLL
    osc->dfllctrl = OSCCTRL_DFLLCTRL_ENABLE;
    while (!(osc->status & OSCCTRL_STATUS_DFLLRDY))
        ;

    // Enable closed loop mode and set tuning parameters
    osc->dfllctrl |= OSCCTRL_DFLLCTRL_MODE;
    osc->dfllmul = OSCCTRL_DFLLMUL_MUL(48000000 / (32 * 1024)) |
                   OSCCTRL_DFLLMUL_CSTEP(0x1f / 4) |
                   OSCCTRL_DFLLMUL_FSTEP(0xff / 4);
    // Wait for lock
    while (!(osc->status & (OSCCTRL_STATUS_DFLLLCKC | OSCCTRL_STATUS_DFLLLCKF)))
        ;

    // Switch to 48Mhz clock, now OSC16M (@4Mhz) isn't used and can be changed.
    gclk_setup(GCLK0, GCLK_GENCTRL_SRC_DFLL48M, 0);

    // Change OSC16M to only activate when used and setup 8Mhz clock for low power mode
    osc->dfllctrl |= OSCCTRL_DFLLCTRL_ONDEMAND;

    osc->osc16mctrl = 0;
    osc->osc16mctrl = OSCCTRL_OSC16MCTRL_ENABLE |
                      OSCCTRL_OSC16MCTRL_FSEL_16Mhz |
                      OSCCTRL_OSC16MCTRL_ONDEMAND;
}


//
// Main initialization
//
int main(int argc, char *argv[])
{
    power_init();

    //
    // Clocks
    //
    clocks_init();
    systick_init(GCLK0_HZ);

    //
    // Start the pulsing LED
    //
    gclk_peripheral_enable(GCLK0, LED_GCLK_PERIPHERAL);
    port_peripheral_enable(LED_PORT, LED_PIN, LED_TC_MUX);
    tc_pwm_init(LED_TC, TC_CTRLA_PRESCALER_DIV1, 1);
    led_handle_work(NULL);

    //
    // Setup the debug UART - Using global clock 0
    //
    gclk_peripheral_enable(GCLK0, DBG_UART_GCLK_PERIPHERAL);
    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_TX_PIN, DBG_UART_PORT_TX_MUX);
    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_RX_PIN, DBG_UART_PORT_RX_MUX);
    sercom_usart_async_init(DBG_UART_ID, &dbg_uart, DBG_UART_PERIPHERAL_ID,
                            GCLK0_HZ, DBG_UART_BAUDRATE,
                            SERCOM_USART_CTRLB_CHSIZE_8BITS, SERCOM_USART_CTRLB_SBMODE_1BIT,
                            SERCOM_USART_CTRLA_FORM_FRAME, SERCOM_USART_CTRLB_PMODE_EVEN,
                            DBG_UART_TX_PAD, DBG_UART_RX_PAD);
    uart_console(&console, &dbg_uart);

    //
    // Setup the I2c port
    //
    gclk_peripheral_enable(GCLK0, TWI_GCLK_PERIPHERAL);
    port_peripheral_enable(TWI_PORT, TWI_PORT_SCL_PIN, TWI_PORT_SCL_MUX);
    port_peripheral_enable(TWI_PORT, TWI_PORT_SDA_PIN, TWI_PORT_SDA_MUX);
    twi_init(&twi, TWI_PERIPHERAL_ID, GCLK0_HZ, TWI_CLKRATE);

    //
    // Setup the USB port
    //
    gclk_peripheral_enable(GCLK0, GCLK_USB);
    port_peripheral_enable(USB_DP_PORT, USB_DP_PIN, USB_DP_MUX);
    port_peripheral_enable(USB_DN_PORT, USB_DN_PIN, USB_DN_MUX);
    usb_init();

    //
    // Setup the control endpoint
    //
    usb_control0_init((char *)&usb_desc, usb_desc_len,
                      (char *)&usb_config, usb_config_len,
                      usb_str_desc);

    //
    // Setup the console, print version info and prompt
    //
    console_init(&console, cmd_table, ARRAY_SIZE(cmd_table),
                 (console_send_t)uart_send_wait, 
                 (console_recv_t)uart_recv,
                 &dbg_uart);
    console_print(&console, "\r\nSAML21-XPLAINED-PRO Ver %d.%d.%d\r\n",
                  VERSION_MAJOR, VERSION_MINOR, VERSION_MICRO);
    console_prompt(&console);

    //
    // Button input
    //
    gclk_peripheral_enable(GCLK0, GCLK_EIC);
    port_pull_enable(BUTTON_PORT, BUTTON_PIN);
    port_pull_direction(BUTTON_PORT, BUTTON_PIN, PORT_PULLUP);
    port_peripheral_enable(BUTTON_PORT, BUTTON_PIN, BUTTON_MUX);
    eic_int_setup(BUTTON_INTNUM, &button, EIC_EDGE_FALL);       // Active low
    eic_int_enable(BUTTON_INTNUM);

    // Only after all external interrupts are configured can it be enabled
    eic_enable();

    //
    // Mainloop
    //
    while (1)
    {
        if (!workqueue_handle_next())
        {
            cpu_sleep();
        }
    }

    return 0;
}

