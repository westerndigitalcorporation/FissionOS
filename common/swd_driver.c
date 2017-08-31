/*
 * swd_driver.c
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

#include "systick.h"

#include "swd.h"
#include "swd_bitbang.h"

#include "swd_driver.h"


/*
 * Platform specific bitbang driver functions, these functions must be
 * implemented by the platform driver.
 */
void swd_bitbang_clk_dir(swd_t *instance, int output)
{
    swd_sam4e_driver_t *driver = instance->driver_private;

    if (output)
    {
        GPIO_SET_OUTPUT(driver->swclk_port, driver->swclk_pin);
    }
    else
    {
        GPIO_SET_INPUT(driver->swclk_port, driver->swclk_pin);
    }
}

void swd_bitbang_clk_set(swd_t *instance, int high)
{
    swd_sam4e_driver_t *driver = instance->driver_private;

    if (high)
    {
        GPIO_SET_HIGH(driver->swclk_port, driver->swclk_pin);
    }
    else
    {
        GPIO_SET_LOW(driver->swclk_port, driver->swclk_pin);
    }
}

void swd_bitbang_dio_set(swd_t *instance, int high)
{
    swd_sam4e_driver_t *driver = instance->driver_private;

    if (high)
    {
        GPIO_SET_HIGH(driver->swdio_port, driver->swdio_pin);
    }
    else
    {
        GPIO_SET_LOW(driver->swdio_port, driver->swdio_pin);
    }
}

void swd_bitbang_dio_dir(swd_t *instance, int output)
{
    swd_sam4e_driver_t *driver = instance->driver_private;

    if (output)
    {
        GPIO_SET_OUTPUT(driver->swdio_port, driver->swdio_pin);
    }
    else
    {
        GPIO_SET_INPUT(driver->swdio_port, driver->swdio_pin);
    }
}

int swd_bitbang_dio_get(swd_t *instance)
{
    swd_sam4e_driver_t *driver = instance->driver_private;

    return GPIO_GET_INPUT(driver->swdio_port, driver->swdio_pin);
}

void swd_bitbang_udelay(uint32_t usecs)
{
    udelay(usecs);
}


/*
 * Platform specific initialization
 */
int swd_sam4e_driver_init(swd_t *instance, swd_sam4e_driver_t *driver)
{
    instance->driver_private = driver;

    // Start and stop sequence functions, from the common bitbang driver
    instance->start = swd_bitbang_start;
    instance->stop = swd_bitbang_stop;
    instance->flush = swd_bitbang_flush;

    // Request functions from the common bitbang driver
    instance->request_recv = swd_bitbang_request_recv;
    instance->request_xmit = swd_bitbang_request_xmit;

    // Enable Pull-Ups
    GPIO_PULLUP_ENABLE(driver->swclk_port, driver->swclk_pin);
    GPIO_PULLUP_ENABLE(driver->swdio_port, driver->swdio_pin);

    // Enable GPIOs
    GPIO_ENABLE(driver->swclk_port, driver->swclk_pin);
    GPIO_ENABLE(driver->swdio_port, driver->swdio_pin);

    return 0;
}


