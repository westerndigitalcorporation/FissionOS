/*
 * sam4_reset.c
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


#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "sam4_clock.h"
#include "sam4_reset.h"
#include "sam4_uart.h"

#include "console.h"

int cmd_reset(uart_drv_t *uart, int argc, char *argv[])
{
    char *reset_reasons[] =
    {
        [RSTC_RSTTYPE_POWERON] = "Power On",
        [RSTC_RSTTYPE_BACKUP] = "Backup",
        [RSTC_RSTTYPE_WATCHDOG] = "Watchdog",
        [RSTC_RSTTYPE_SOFTWARE] = "Software",
        [RSTC_RSTTYPE_EXTERNAL] = "External",
    };

    if ((argc == 2) && !strcmp(argv[1], "now"))
    {
        RESET();
    }

    console_print("Reset Reason: %s\r\n",
                  reset_reasons[RSTC_SR_RSTTYPE(RSTC->sr)]);

    return 0;
}

