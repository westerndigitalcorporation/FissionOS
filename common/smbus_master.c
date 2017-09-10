/*
 * smbus_master.c
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


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "saml_sercom.h"

#include <vectors.h>
#include <console.h>

#include "saml_vectors.h"
#include "crc.h"

#include "smbus_master.h"


extern twi_drv_t *twi_drv[SERCOM_COUNT];

void cmd_smbus_result(twi_drv_t *t, void *arg, int result)
{
    int *result_ptr = (int *)arg;

    *result_ptr = -result;
}

#define TWI_CMD_BUFLEN                           32
int cmd_smbus(console_t *console, int argc, char *argv[])
{
    uint32_t read_len = 0, write_len = 0;
    uint8_t write_buffer[TWI_CMD_BUFLEN], read_buffer[TWI_CMD_BUFLEN];
    int count, twi_index, i, result;
    uint8_t addr;
    char *result_msg[] =
    {
        "Success",
        "NACK",
        "Bus Lost",
    };

    // cmd, device number, i2c addr, read len, write bytes...
    if (argc <= 3)
    {
        if ((argc == 2) && (!strcmp(argv[1], "list")))
        {
            console_print(console, "Available devices\r\n");
            for (i = 0; i < SERCOM_COUNT; i++)
            {
                if (twi_drv[i])
                {
                    console_print(console, "  %d\r\n", i);
                }
            }

            return 0;
        }

        cmd_help_usage(console, argv[0]);
        return 0;
    }

    count = 1;

    // TWI Index parameter
    twi_index = strtoul(argv[count++], NULL, 0);
    if (twi_index >= SERCOM_COUNT)
    {
        console_print(console, "Invalid TWI device\r\n");
        return 0;
    }

    if (!twi_drv[twi_index])
    {
        console_print(console, "TWI device not initialized\r\n");
        return 0;
    }

    // Destination Addr
    addr = strtoul(argv[count++], NULL, 0);

    // Read length parameter
    read_len = strtoul(argv[count++], NULL, 0);
    if (read_len)
    {
        read_len++;
    }
    if (read_len > sizeof(read_buffer))
    {
        console_print(console, "Read length must be <= %d\r\n", sizeof(read_buffer));
        return 0;
    }

    // Write data parameters
    while ((count < argc) && (write_len < (sizeof(write_buffer) - 1)))
    {
        write_buffer[write_len++] = strtoul(argv[count++], NULL, 0);
    }

    if (count < argc)
    {
        console_print(console, "Write length must be <= %d\r\n", sizeof(write_buffer) - 1);
        return -1;
    }

    if (write_len)
    {
        // Calculate the SMBus PEC
        write_buffer[write_len++] = crc8(write_buffer, count);
    }

    if (twi_master_xfer(twi_drv[twi_index], addr, write_buffer, write_len, read_buffer, read_len,
                        cmd_smbus_result, &result))
    {
        console_print(console, "TWI Controller Busy");
        return -1;
    }

    twi_master_wait(twi_drv[twi_index]);

    console_print(console, "Result: %s\r\n", result_msg[result]);
    if (!result && read_len)
    {
        uint8_t pec;

        // Validate SMBus PEC
        pec = crc8(read_buffer, read_len - 1);
        if (pec != read_buffer[read_len - 1])
        {
            console_print(console, "Incorrect PEC\r\n");
        }
        else
        {
            for (i = 0; i < read_len; i++)
            {
                console_print(console, "%02x ", read_buffer[i]);
            }
            console_print(console, "\r\n");
        }
    }

    return 0;
}

