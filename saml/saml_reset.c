/*
 * saml_reset.c
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

#include "saml_sercom.h"

#include <console.h>

#include "saml_reset.h"


int cmd_reset(console_t *console, int argc, char *argv[])
{
    if (argc < 2)
    {
        cmd_help_usage(console, argv[0]);
        return 0;
    }

    if (!strcmp(argv[1], "show"))
    {
        char *reason = "", *reasons[] = {
            "POR",
            "BOD12",
            "BOD33",
            "",
            "EXT",
            "WDT",
            "SYST",
            "BACKUP",
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(reasons); i++)
        {
            if (RESET->rcause & (1 << i))
            {
                reason = reasons[i];
                break;
            }
        }

        console_print(console, "Reset Reason: %s\r\n", reason);

        return 0;
    }
    else if (!strcmp(argv[1], "app"))
    {
        RESET_CONFIG = RESET_CONFIG_APPLICATION;

        saml_soft_reset();

        return 0;
    }
    else if (!strcmp(argv[1], "bootloader"))
    {
        RESET_CONFIG = RESET_CONFIG_BOOTLOADER;

        saml_soft_reset();

        return 0;
    }

    cmd_help_usage(console, argv[0]);

    return 0;
}

