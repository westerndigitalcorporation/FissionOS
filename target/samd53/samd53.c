/*
 * samd53.c
 *
 * Copyright (c) 2017-2018 Jeremy Garff
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
 * Author: Jeremy Garff <jer@jers.net>
 *
 */


#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "saml_clocks.h"
#include "saml_port.h"
#include "saml_sercom.h"
#include "saml_nvm.h"
#include "saml_reset.h"
#include "saml_usb.h"
#include "saml_tcc.h"
#include "saml_adc.h"
#include "saml_power.h"
#include "saml_arch.h"

#include <vectors.h>
#include <systick.h>
#include <workqueue.h>
#include <console.h>
#include <usb.h>
#include <adc_calc.h>

#include "usb_config.h"
#include "usb_serial.h"
#include "usb_vendor.h"

#include "samd53_version.h"
#include "samd53.h"


console_t console;


//
// Stub for linking.  Not used without full task support.
//
void thread_switch_handler(void)
{
}

int cmd_status(console_t *console, int argc, char *argv[]);
//
// Console Commands
//
uart_drv_t dbg_uart;
cmd_entry_t cmd_table[] =
{
    CONSOLE_CMD_HELP,
    CONSOLE_CMD_NVM,
    //CONSOLE_CMD_RESET,
    {
        .cmdstr = "status",
        .callback = cmd_status,
        .usage = "  status show\r\n",
        .help =
            "  Show values of the analog channels.\r\n"
            "    show       : Show values in mV.\r\n"
    },
    CONSOLE_CMD_USB,
};


adc_desc_t adc0;
const adc_calc_voltage_divider_t adc_calc_10_1_divider =
{
    .mvref = 1000,
    .r1 = 10000,
    .r2 = 1000,
};
uint32_t adc0_ain0_value;
adc_drv_t adc0_ain0_vsense =
{
    .value = &adc0_ain0_value,
    .channel = 0,
    .calc = adc_calc_divider_12bit_unsigned,
    .calc_arg = (void *)&adc_calc_10_1_divider,
};

uint32_t adc0_ain1_value;
adc_drv_t adc0_ain1_vsense =
{
    .value = &adc0_ain1_value,
    .channel = 1,
    .calc = adc_calc_divider_12bit_unsigned,
    .calc_arg = (void *)&adc_calc_10_1_divider,
};
uint32_t adc0_ain2_value;
adc_drv_t adc0_ain2_vsense =
{
    .value = &adc0_ain2_value,
    .channel = 2,
    .calc = adc_calc_divider_12bit_unsigned,
    .calc_arg = (void *)&adc_calc_10_1_divider,
};
uint32_t adc0_ain3_value;
adc_drv_t adc0_ain3_vsense =
{
    .value = &adc0_ain3_value,
    .channel = 3,
    .calc = adc_calc_divider_12bit_unsigned,
    .calc_arg = (void *)&adc_calc_10_1_divider,
};

adc_drv_t *adc0_sensors[] =
{
    &adc0_ain0_vsense,
    &adc0_ain1_vsense,
    &adc0_ain2_vsense,
    &adc0_ain3_vsense,
};

uint32_t *adc_values[] =
{
    &adc0_ain0_value,
    &adc0_ain1_value,
    &adc0_ain2_value,
    &adc0_ain3_value,
};

void adc_complete(adc_drv_t **adc, uint32_t count, void *arg)
{
    workqueue_t *wq = (workqueue_t *)arg;
    workqueue_add(wq, 0);
}

adc_queue_entry_t adc_queue = 
{
    .adcs = adc0_sensors,
    .count = ARRAY_SIZE(adc0_sensors),
    .complete = adc_complete,
};

uint32_t samples = 0;
void adc_worker(void *arg);
workqueue_t adc_wq =
{
    .callback = adc_worker,
    .arg = NULL,
};
void adc_worker(void *arg)
{
    int i;

    adc_queue.complete_arg = &adc_wq;
    adc_start(&adc0, &adc_queue);
    samples++;

    for (i = 0; i < ARRAY_SIZE(adc_values); i++)
    {
        volatile int irqstate = irq_save();
        uint32_t value = (*adc_values[i] * TCC0_MAX) / 10000;  // Convert mV to percent of TCC0_MAX

        tcc_pwm_duty(TCC0, i, value);

        irq_restore(irqstate);
    }
    
}

void status_worker(void *arg);
workqueue_t status_wq =
{
    .callback = status_worker,
    .arg = NULL,
};

int status_direction = 1;
int status_value = 0;
void status_worker(void *arg)
{
    if (!status_direction)
    {
        status_value--;
        if (!status_value)
        {
            status_direction = 1;
        }
    }
    else
    {
        status_value++;
        if (status_value >= (STATUS_MAX - 1))
        {
            status_direction = 0;
        }
    }

    tcc_pwm_duty(LED_TCC, LED_TCC_CHANNEL, (status_value * LED_TCC_MAX) / STATUS_MAX); 

    workqueue_add(&status_wq, (SYSTICK_FREQ * (1000 / STATUS_MAX)) / 1000);
}

int cmd_status(console_t *console, int argc, char *argv[])
{
    if ((argc == 2) && !strcmp(argv[1], "show"))
    {
        int i;

        console_print(console, "VBUS  : %s\r\n", port_get(VBUS_PORT, VBUS_PIN) ? "High" : "Low");
        console_print(console, "Status: (%d samples taken)\r\n", samples * 64);
        for (i = 0; i < ARRAY_SIZE(adc_values); i++)
        {
            int value = *adc_values[i];
            console_print(console, "  %2d (%05d): %d.%03d\r\n", 
                          i, value, value / 1000, value % 1000);
        }
    }
    else
    {
        cmd_help_usage(console, argv[0]);
    }

    return 0;
}

void vbus_callback(void *arg)
{
    if (port_get(VBUS_PORT, VBUS_PIN))
    {
        usb_attach();
    }
    else
    {
        usb_detach();
    }
}

ext_int_t vbus =
{
    .callback = vbus_callback,
    .arg = NULL,
};

void clock_init(void)
{
    volatile oscctrl_t *osc = OSCCTRL;
    volatile xosc32k_t *osc32 = XOSC32K;
    uint32_t tmp32;
    uint16_t tmp16;
    uint8_t tmp8;

    /*
     *
     * Setup the DFLL for full speed using closed loop mode referenced
     * from a external 32Khz crystal.  Then use that to generate the 120Mhz CPU
     * clock.
     *
     * OSC32K --> GCLK1 --> DFLL (48Mhz) --> GCLK3 (2Mhz) --> DPLL0 (120Mhz)
     *                           |                                  |
     *                           --> GCLK2 (48Mhz)                  --> GCLK0 (120Mhz) --> CPU
     *
     * The following are the clock sources available to peripherals after configuration:
     *
     * GCLK0 - 120Mhz
     * GCLK1 - 32Khz
     * GCLK2 - 48Mhz
     * GCLK3 - 2Mhz
     *
     */

    //
    // 32Khz XOSC
    //
    // Setup XOSC32 from external crystal
    tmp16 = XOSC32K_XOSC32K_XTALEN |
            XOSC32K_XOSC32K_EN32K |
            XOSC32K_XOSC32K_STARTUP(2) |
            XOSC32K_XOSC32K_CGM_HS;
    write16(&osc32->xosc32k, tmp16);

    tmp16 |= XOSC32K_XOSC32K_ENABLE;
    write16(&osc32->xosc32k, tmp16);
    while (!(osc32->status & XOSC32K_STATUS_XOSC32KRDY))
        ;
    
    // Loop back the 32K reference to the DFLL
    gclk_setup(LOWSPEED_GCLK, GCLK_GENCTRL_SRC_XOSC32K, 0);
    gclk_peripheral_enable(LOWSPEED_GCLK, GCLK_DFLL48M_REF);

    // Setup the CPU to run off the slow clock temporarily while we setup
    // the DFLL in closed loop mode
    gclk_setup(CLK_CORE, GCLK_GENCTRL_SRC_XOSC32K, 0);

    //
    // 48 Mhz DFLL Config
    //
    // Turn off first
    tmp8 = 0;
    write8(&osc->dfllctrla, tmp8);

    // See datasheet for multiplier values
    tmp32 = OSCCTRL_DFLLMUL_MUL(1464) | OSCCTRL_DFLLVAL_COARSE(1) |
          OSCCTRL_DFLLVAL_DIFF(1);
    write32(&osc->dfllmul, tmp32);

    tmp8 = OSCCTRL_DFLLCTRLB_MODE;  // Closed loop mode
    write8(&osc->dfllctrlb, tmp8);

    // Turn it on
    write8(&osc->dfllctrla, OSCCTRL_DFLLCTRLA_ENABLE);
    while (!(osc->status & OSCCTRL_STATUS_DFLLRDY))
        ;

    // Setup the 48Mhz clock source
    gclk_setup(CLK48MHZ_GCLK, GCLK_GENCTRL_SRC_DFLL, 0);
    
    //
    // 2Mhz GCLK
    //
    // Run the 120Mhz clock from the 48Mhz reference
    gclk_setup(CLK2MHZ_GCLK, GCLK_GENCTRL_SRC_DFLL, 24);  // 2 Mhz

    //
    // 120 Mhz DPLL0 Config
    //
    // Set the DPLL0 as a output from the 2Mhz clock for reference
    gclk_peripheral_enable(CLK2MHZ_GCLK, GCLK_FDPLL0);

    // Turn the PLL off
    write8(&osc->dpll0ctrla, 0);
    while ((osc->dpll0syncbusy & OSCCTRL_DPLL0SYNCBUSY_ENABLE))
        ;

    // Setup the PLL to use the 2Mhz GCLK reference from above, and multiply
    // by 60 to get to 120Mhz
    write32(&osc->dpll0ctrlb, OSCCTRL_DPLL0CTRLB_REFCLK_GCLK);
    write32(&osc->dpll0ratio, OSCCTRL_DPLL0RATIO_LDR(60 - 1));
    while ((osc->dpll0syncbusy & OSCCTRL_DPLL0SYNCBUSY_DPLLRATIO))
        ;

    // Fire it up and wait for register sync
    write8(&osc->dpll0ctrla, OSCCTRL_DPLL0CTRLA_ENABLE);
    while ((osc->dpll0syncbusy & OSCCTRL_DPLL0SYNCBUSY_ENABLE))
        ;

    // Wait for the clock to lock and become ready
    while (!(osc->dpll0status & OSCCTRL_DPLL0STATUS_CLKRDY))
        ;

    // Set the CPU to use the DPLL at 120Mhz
    gclk_setup(CLK_CORE, GCLK_GENCTRL_SRC_DPLL0, 0);
}

//
// Main initialization
//
int main(int argc, char *argv[])
{
    volatile mclk_t *mclk = MCLK;

    systick_init(GCLK0_HZ);
    clock_init();

    //
    // Setup the UART
    //
    mclk->apbamask |= MCLK_APBAMASK_SERCOM0;
    gclk_peripheral_enable(CLK48MHZ_GCLK, DBG_UART_GCLK_CORE);
    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_TX_PIN, DBG_UART_PORT_TX_MUX);
    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_RX_PIN, DBG_UART_PORT_RX_MUX);
    sercom_usart_async_init(DBG_UART_ID, &dbg_uart, DBG_UART_PERIPHERAL_ID,
                            UART_SRC_HZ, DBG_UART_BAUDRATE,
                            SERCOM_USART_CTRLB_CHSIZE_8BITS,
                            SERCOM_USART_CTRLB_SBMODE_1BIT,
                            SERCOM_USART_CTRLA_FORM_FRAME, SERCOM_USART_CTRLB_PMODE_EVEN,
                            DBG_UART_TX_PAD, DBG_UART_RX_PAD);
    // UART debugging
#ifdef UART_CONSOLE
    uart_console(&console, &dbg_uart);
#endif
    
    //
    // Setup external interrupts
    //
    mclk->apbamask |= MCLK_APBAMASK_EIC;
    gclk_peripheral_enable(CLK48MHZ_GCLK, GCLK_EIC);
    port_peripheral_enable(VBUS_PORT, VBUS_PIN, VBUS_MUX);
    port_dir(VBUS_PORT, VBUS_PIN, 0);  // Input

    eic_int_setup(VBUS_INTNUM, &vbus, EIC_EDGE_BOTH);
    eic_int_enable(VBUS_INTNUM);

    eic_enable();

    //
    // Setup ADC
    //
    mclk->apbdmask |= MCLK_APBDMASK_ADC0;
    // Enable the internal 1v0 bandgap reference
    SUPC->vref = SUPC_VREF_VREFOE | SUPC_VREF_ONDEMAND;
    // Setup clocks and pins
    gclk_peripheral_enable(CLK48MHZ_GCLK, GCLK_ADC0);
    port_peripheral_enable(ADC0_AIN0_PORT, ADC0_AIN0_PIN, ADC_MUX);
    port_peripheral_enable(ADC0_AIN1_PORT, ADC0_AIN1_PIN, ADC_MUX);
    port_peripheral_enable(ADC0_AIN2_PORT, ADC0_AIN2_PIN, ADC_MUX);
    port_peripheral_enable(ADC0_AIN3_PORT, ADC0_AIN3_PIN, ADC_MUX);

    adc_init(0, &adc0);

    //
    // Setup PWM
    //
    mclk->apbbmask |= MCLK_APBBMASK_TCC0;
    gclk_peripheral_enable(CLK48MHZ_GCLK, GCLK_TCC0_1);
    port_peripheral_enable(PWM0_PORT, PWM0_PIN, PWM_MUX);
    port_peripheral_enable(PWM1_PORT, PWM1_PIN, PWM_MUX);
    port_peripheral_enable(PWM2_PORT, PWM2_PIN, PWM_MUX);
    port_peripheral_enable(PWM3_PORT, PWM3_PIN, PWM_MUX);

    tcc_pwm_init(TCC0, TCC_CTRLA_PRESCALER_DIV1, 0, TCC0_MAX - 2);

    //
    // Turn on the output enable
    //
    port_peripheral_disable(OE_PORT, OE_PIN);
    port_dir(OE_PORT, OE_PIN, 1);  // Output
    port_set(OE_PORT, OE_PIN, 1);  // High

    //
    // Setup the USB port
    //
    mclk->apbbmask |= MCLK_APBBMASK_USB;
    gclk_peripheral_enable(CLK48MHZ_GCLK, GCLK_USB);
    port_peripheral_enable(USB_DP_PORT, USB_DP_PIN, USB_DP_MUX);
    port_peripheral_enable(USB_DN_PORT, USB_DN_PIN, USB_DN_MUX);
    usb_init();

    //
    // Setup the control and serial endpoints
    //
#ifndef UART_CONSOLE
    usb_serial_init(usb_console_rx_callback, &console);
#endif
    usb_control0_init((char *)&usb_desc, usb_desc_len,
                      (char *)&usb_config, usb_config_len,
                      usb_str_desc);
    usb_vendor_init();

    //
    // Setup the console
    //
    // USB Console
#ifndef UART_CONSOLE
    console_init(&console, cmd_table, ARRAY_SIZE(cmd_table),
                 (console_send_t)usb_console_send_wait, 
                 (console_recv_t)usb_console_recv,
                 NULL);
#else
    // Serial Console Debugging
    console_init(&console, cmd_table, ARRAY_SIZE(cmd_table),
                 (console_send_t)uart_send_wait, 
                 (console_recv_t)uart_recv,
                 &dbg_uart);
#endif /* UART_CONSOLE */

    //
    // Status LED
    //
    // Turn on the status LED
    mclk->apbbmask |= MCLK_APBBMASK_TCC1;
    port_strength(LED_PORT, LED_PIN, PORT_DRIVE_HIGH);
    port_peripheral_enable(LED_PORT, LED_PIN, LED_MUX);

    tcc_pwm_init(LED_TCC, TCC_CTRLA_PRESCALER_DIV1, (1 << LED_TCC_CHANNEL), LED_TCC_MAX - 2);
    status_worker(NULL);

    // Setup the USB state
    vbus_callback(NULL);

    // Start the ADC sampler and PWM generator based on levels
    adc_worker(NULL);

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

