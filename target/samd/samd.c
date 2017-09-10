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

#include "samd_version.h"
#include "samd.h"

#include "samd_xmodem.h"


extern uint32_t __config_word;

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
    CONSOLE_CMD_RESET,
    CONSOLE_CMD_UPLOAD,
};


void clock_init(void)
{
    volatile uint32_t tmp;

    // CAUTION: Assuming 3.3 volt operation.
    // Flash must be configured for 1 wait state @3v3 before going full speed.
    // At 1v8, 3 wait states must be used.
    // Turn on the 32khz oscillator
    NVMCTRL->ctrlb = NVMCTRL_CTRLB_MANW | NVMCTRL_CTRLB_RWS(3);

    tmp = SYSCTRL_XOSC32K_EN32K |
          SYSCTRL_XOSC32K_XTALEN;
    SYSCTRL->xosc32k = tmp;

    tmp |= SYSCTRL_XOSC32K_ENABLE;
    SYSCTRL->xosc32k = tmp;
    // Wait for startup
    while (!(SYSCTRL->pclksr & SYSCTRL_PCLKSR_XOSC32KRDY))
        ;

    // Hook the 32Khz to generic clock 1 and then to the reference clock source
    gclk_setup(GCLK1, GCLK_GENCTRL_SRC_XOSC32K, 0);
    gclk_peripheral_enable(GCLK1, GCLK_DFLL48M_REF);

    // Turn on the DLL and wait for ready
    tmp = SYSCTRL_DFLLCTRL_ENABLE;
    SYSCTRL->dfllctrl = tmp;
    while (!(SYSCTRL->pclksr & SYSCTRL_PCLKSR_DFLLRDY))
        ;

    // Setup the coarse step size dfllmul.cstep dfllmul.fstep
    // Setup the multiplication factor dfllmul.mul
    tmp = SYSCTRL->dfllctrl | SYSCTRL_DFLLCTRL_MODE;
    SYSCTRL->dfllctrl = tmp;
    while (!(SYSCTRL->pclksr & SYSCTRL_PCLKSR_DFLLRDY))
        ;

    tmp = SYSCTRL_DFLLMUL_MUL(48000000 / 32768) |
          SYSCTRL_DFLLMUL_FSTEP(0xff / 4) |
          SYSCTRL_DFLLMUL_CSTEP(0x1f / 4);
    SYSCTRL->dfllmul = tmp;
    while (!(SYSCTRL->pclksr & (SYSCTRL_PCLKSR_DFLLLCKC | SYSCTRL_PCLKSR_DFLLLCKF)))
        ;

    // Switch to the DFLL clock for the main frequency
    gclk_setup(GCLK0, GCLK_GENCTRL_SRC_DFLL48M, 0);
}


//
// Main initialization
//
int main(int argc, char *argv[])
{
    clock_init();

    systick_init(GCLK0_HZ);

    // Setup the debug UART
    gclk_peripheral_enable(CLK_CORE, DBG_UART_GCLK_CORE);
	PM->apbcmask |= PM_APBCMASK_SERCOM5;

    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_TX_PIN, DBG_UART_PORT_TX_MUX);
    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_RX_PIN, DBG_UART_PORT_RX_MUX);

    sercom_usart_async_init(DBG_UART_ID, &dbg_uart, DBG_UART_PERIPHERAL_ID,
                            GCLK0_HZ, DBG_UART_BAUDRATE,
                            SERCOM_USART_CTRLB_CHSIZE_8BITS, SERCOM_USART_CTRLB_SBMODE_1BIT,
                            SERCOM_USART_CTRLA_FORM_FRAME, SERCOM_USART_CTRLB_PMODE_EVEN,
                            DBG_UART_TX_PAD, DBG_UART_RX_PAD);
    uart_console(&console, &dbg_uart);

    // Setup the console, print version info and prompt
    console_init(&console, cmd_table, ARRAY_SIZE(cmd_table),
                 (console_send_t)uart_send_wait, 
                 (console_recv_t)uart_recv,
                 &dbg_uart);
    console_print(&console, "\r\nSAMD20-KingsPeak Ver %d.%d.%d\r\n",
                  VERSION_MAJOR, VERSION_MINOR, VERSION_MICRO);
    console_prompt(&console);

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

