/*
 * swd_gdb_backend.c
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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "swd.h"
#include "swd_target.h"
#include "swd_gdb_backend.h"

#include "sam4_reset.h"

#include "cortexm_debug.h"


#define GDB_START_CHAR                           '$'                 //< GDB Start Command
#define GDB_END_CHAR                             '#'                 //< GDB End Command

#define GDB_COMMAND_SIZE                         128                 //< Largest recv command buffer size
#define GDB_CSUM_SIZE                            2                   //< Cksum size in bytes

/*
 * GDB Target States
 */
#define GDB_TARGET_HALTED                        0                   //< Target CPU Halted
#define GDB_TARGET_RUNNING                       1                   //< Target CPU Running

/*
 * Client Stream parsing states
 */
#define GDB_STREAM_STATE_IDLE                    0
#define GDB_STREAM_STATE_DATA                    1
#define GDB_STREAM_STATE_CSUM                    2
#define GDB_STREAM_STATE_EXIT                    3

#define GDB_PSR_REG_INDEX                        25
#define GDB_MONITOR_CMDSTR_MAX                   80
#define GDB_CSUMSTR_MAX                          10

typedef struct hw_bp                                                 //< Hardware Breakpoint Descriptor
{
    uint32_t flags;
#define HW_BP_FLAGS_SET                          0x1
    uint32_t addr;
} hw_bp_t;
hw_bp_t hw_bps[HW_BP_MAX];

typedef struct swd_gdb_client swd_gdb_client_t;                      //< Platform specific client handler

static uint8_t gdb_stream_state;
static uint8_t gdb_command[GDB_COMMAND_SIZE];
static uint8_t gdb_command_len;
static uint8_t gdb_csum_len;
static uint8_t gdb_pending_reset = 0;
static int gdb_target_state = GDB_TARGET_HALTED;
static char *gdb_start_seq = "+$";                                   //< GDB Start Reply String


/**
 * Set hardware breakpoint.
 *
 * @param    instance   SWD Driver Instance pointer.
 * @param    addr       Execution Address for Exception.
 *
 * @returns  0 on success, negative error on failure.
 */
int hw_bp_insert(swd_t *instance, uint32_t addr)
{
    int i;

    for (i = 0; i < HW_BP_MAX; i++)
    {
        if (!(hw_bps[i].flags & HW_BP_FLAGS_SET))
        {
            break;
        }
    }

    if (i == HW_BP_MAX)
    {
        return -1;
    }

    hw_bps[i].flags |= HW_BP_FLAGS_SET;
    hw_bps[i].addr = addr;

    swd_target_breakpoint_set(instance, i, addr);

    return 0;
}

/**
 * Clear hardware breakpoint.
 *
 * @param    instance   SWD Driver Instance pointer.
 * @param    addr       Execution Address for Exception.
 *
 * @returns  0 on success, negative error on failure.
 */
int hw_bp_remove(swd_t *instance, uint32_t addr)
{
    int i;

    for (i = 0; i < HW_BP_MAX; i++)
    {
        if (hw_bps[i].flags & HW_BP_FLAGS_SET)
        {
            if (hw_bps[i].addr == addr)
            {
                hw_bps[i].flags &= ~HW_BP_FLAGS_SET;

                swd_target_breakpoint_clear(instance, i);

                return 0;
            }
        }
    }

    return -1;
}

/**
 * Convert a hexadecimal string (2-bytes) into a 8-bit value.
 *
 * @param    hexstr   Pre-parsed and assumed valid hex string.
 *
 * @returns  8-bit converted value.
 */
uint8_t hexstr_to_uint8(const char *hexstr)
{
    char val = 0;
    int i;

    for (i = 0 ; i < (sizeof(uint8_t) * 2); i++)
    {
        if (!hexstr[i])
        {
            break;
        }

        val <<= 4;

        if ((hexstr[i] >= '0') && (hexstr[i] <= '9'))
        {
            val |= hexstr[i] - '0';
        }
        else if ((hexstr[i] >= 'a') && (hexstr[i] <= 'f'))
        {
            val |= hexstr[i] - 'a' + 0xa;
        }
        else if ((hexstr[i] >= 'A') && (hexstr[i] <= 'F'))
        {
            val |= hexstr[i] - 'A' + 0xa;
        }
    }

    return val;
}

/**
 * Convert a hexadecimal string (8-bytes) into a 32-bit value.
 *
 * @param    hexstr   Pre-parsed and assumed valid hex string.
 *
 * @returns  32-bit converted value.
 */
uint32_t hexstr_to_uint32(const char *hexstr)
{
    uint32_t val = 0;
    int i;

    for (i = 0 ; i < (sizeof(uint32_t) * 2); i++)
    {
        if (!hexstr[i])
        {
            break;
        }

        val <<= 4;

        if ((hexstr[i] >= '0') && (hexstr[i] <= '9'))
        {
            val |= hexstr[i] - '0';
        }
        else if ((hexstr[i] >= 'a') && (hexstr[i] <= 'f'))
        {
            val |= hexstr[i] - 'a' + 0xa;
        }
        else if ((hexstr[i] >= 'A') && (hexstr[i] <= 'F'))
        {
            val |= hexstr[i] - 'A' + 0xa;
        }
    }

    return val;
}

/**
 * Convert and write a 32-bit data value to the connected client.  Output is
 * sent as a hexadecimal string.  Updates and returns csum.
 *
 * @param    client    Client instance pointer.
 * @param    data      Data to convert and send.
 * @param    csum      Previous checksum.
 *
 * @returns  New checksum accounting for transmitted data.
 */
uint8_t gdb_dump_uint32(swd_gdb_client_t *client, uint32_t data, uint8_t csum)
{
    char tmpbuf[GDB_CSUMSTR_MAX];
    int j;

    for (j = 0; j < sizeof(uint32_t); j++)
    {
        uint8_t tmp = ((uint8_t *)&data)[j];

        sprintf(tmpbuf, "%02x", tmp);
        csum += tmpbuf[0] + tmpbuf[1];
        swd_gdb_write(client, tmpbuf, (uint32_t)strlen(tmpbuf));
    }

    return csum;
}

/**
 * Convert and write a 8-bit data value to the connected client.  Output is
 * sent as a hexadecimal string.  Updates and returns csum.
 *
 * @param    client    Client instance pointer.
 * @param    data      Data to convert and send.
 * @param    csum      Previous checksum.
 *
 * @returns  New checksum accounting for transmitted data.
 */
uint8_t gdb_dump_uint8(swd_gdb_client_t *client, uint8_t data, uint8_t csum)
{
    char tmpbuf[GDB_CSUMSTR_MAX];

    sprintf(tmpbuf, "%02x", data);
    csum += tmpbuf[0] + tmpbuf[1];
    swd_gdb_write(client, tmpbuf, (uint32_t)strlen(tmpbuf));

    return csum;
}

/**
 * Given a string, split and return two strings which were separated by a
 * comma.  In order to save memory on embedded devices, does not allocate
 * new strings, and instead modified existing string.
 *
 * @param    raw      Raw input string, will be modified.
 * @param    field1   First string before comma.
 * @param    field2   Second string after comma.
 *
 * @returns  None
 */
void gdb_get_uint32_fields(char *raw, uint32_t *field1, uint32_t *field2)
{
    char *field1str = raw;
    char *field2str = NULL;
    uint32_t i;

    for (i = 0; i < strlen(field1str); i++)
    {
        if (field1str[i] == ',')
        {
            field2str = &field1str[i + 1];
            field1str[i] = 0;
            break;
        }
    }

    *field1 = hexstr_to_uint32(field1str);
    if (field2str)
    {
        *field2 = hexstr_to_uint32(field2str);
    }
}

/**
 * Send reply string to client.  Append calculated checksum.
 *
 * @param    client    Client instance pointer.
 * @param    reply     String to send.
 *
 * @returns  None
 */
void gdb_reply(swd_gdb_client_t *client, char *reply)
{
    uint16_t csum = 0;
    char tmp[GDB_CSUMSTR_MAX];
    uint32_t i;

    for (i = 0; i < strlen(reply); i++)
    {
        csum += reply[i];
    }
    sprintf(tmp, "#%02x", csum);

    swd_gdb_write(client, gdb_start_seq, (uint32_t)strlen(gdb_start_seq));
    swd_gdb_write(client, reply, (uint32_t)strlen(reply));
    swd_gdb_write(client, tmp, (uint32_t)strlen(tmp));
}

/**
 * Parse and handle set breakpoint command from client.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 * @param    str       String from client.
 *
 * @returns  0 on success, negative error number on failure.
 */
int gdb_breakpoint_set(swd_gdb_client_t *client, swd_t *instance, char *str)
{
    uint32_t addr, type;

    if (strlen(str) <= 2)
        return -1;

    gdb_get_uint32_fields(&str[2], &addr, &type);
    return hw_bp_insert(instance, addr);
}

/**
 * Parse and handle clear breakpoint command from client.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 * @param    str       String from client.
 *
 * @returns  0 on success, negative error number on failure.
 */
int gdb_breakpoint_clear(swd_gdb_client_t *client, swd_t *instance, char *str)
{
    uint32_t addr, type;

    if (strlen(str) <= 2)
        return -1;

    gdb_get_uint32_fields(&str[2], &addr, &type);
    return hw_bp_remove(instance, addr);
}

/**
 * Get and return CPU registers to client.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 *
 * @returns  0 on success, negative error number on failure.
 */
int gdb_dump_registers(swd_gdb_client_t *client, swd_t *instance)
{
    uint32_t target_regs[CORTEX_MAX_REGS];
    char tmpbuf[GDB_CSUMSTR_MAX];
    uint8_t csum = 0;
    int i, ret;

    ret = swd_target_is_halted(instance);
    if (ret <= 0)
    {
        gdb_reply(client, "E00");
        return -1;
    }

    ret = swd_target_regs(instance, target_regs);
    if (ret)
    {
        gdb_reply(client, "E00");
        return -1;
    }

    swd_gdb_write(client, gdb_start_seq, (uint32_t)strlen(gdb_start_seq));
    for (i = 0; i <= CORTEX_REG_PC; i++)
    {
        csum = gdb_dump_uint32(client, target_regs[i], csum);
    }

    for (i = 0; i < 8 * 3; i++)   // Fake out floating point regs
    {
        csum = gdb_dump_uint32(client, 0, csum);
    }
    csum = gdb_dump_uint32(client, 0, csum);  // No FPS
    csum = gdb_dump_uint32(client, target_regs[CORTEX_REG_xPSR], csum);

    sprintf(tmpbuf, "#%02x", csum);
    swd_gdb_write(client, tmpbuf, (uint32_t)strlen(tmpbuf));

    return 0;
}

/**
 * Parse and set CPU register command from client.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 * @param    regstr    GDB register string to parse.
 *
 * @returns  0 on success, negative error number on failure.
 */
int gdb_set_register(swd_gdb_client_t *client, swd_t *instance, char *regstr)
{
    char *valstr = regstr;
    uint32_t regval;
    uint8_t reg;

    while (valstr)
    {
        if (*valstr == '=')
        {
            *valstr++ = 0;
            break;
        }
        valstr++;
    }

    reg = hexstr_to_uint8(regstr);
    regval = hexstr_to_uint32(valstr);

    if (reg == GDB_PSR_REG_INDEX)
    {
        reg = CORTEX_REG_xPSR;
    }

    return swd_target_set_reg(instance, reg, regval);
}

/**
 * Parse and set memory command from client.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 * @param    addrlen   GDB memory/length string to parse.
 *
 * @returns  0 on success, negative error number on failure.
 */
int gdb_write_memory(swd_gdb_client_t *client, swd_t *instance, char *addrlen)
{
    char *data = addrlen;
    uint32_t addr, len;

    while (*data)
    {
        if (*data == ':')
        {
            *data++ = 0;
            break;
        }
        data++;
    }

    gdb_get_uint32_fields(addrlen, &addr, &len);

    while (data[0] && data[1])
    {
        uint8_t d = hexstr_to_uint8(data);

        if (swd_target_write_uint8(instance, addr++, d))
        {
            return -1;
        }

        data += 2;
    }

    return 0;
}

/**
 * Parse and read memory command from client.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 * @param    addrlen   GDB memory/length string to parse.
 *
 * @returns  0 on success, negative error number on failure.
 */
int gdb_read_memory(swd_gdb_client_t *client, swd_t *instance, char *addrlen)
{
    uint32_t addr, len = 1;
    uint8_t csum = 0;
    char tmpbuf[GDB_CSUMSTR_MAX];
    uint32_t i;

    gdb_get_uint32_fields(addrlen, &addr, &len);

    swd_gdb_write(client, gdb_start_seq, (uint32_t)strlen(gdb_start_seq));
    for (i = 0; i < len; i++)
    {
        uint8_t data;

        swd_target_read_uint8(instance, addr + i, &data);
        csum = gdb_dump_uint8(client, data, csum);
    }

    sprintf(tmpbuf, "#%02x", csum);
    swd_gdb_write(client, tmpbuf, (uint32_t)strlen(tmpbuf));

    return 0;
}

/**
 * Resume CPU and reply to client appropriately.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 *
 * @returns  None
 */
void gdb_resume(swd_gdb_client_t *client, swd_t *instance)
{
    gdb_target_state = GDB_TARGET_RUNNING;
    if (gdb_pending_reset)
    {
        swd_target_reset(instance);
        gdb_pending_reset = 0;
    }
    else
    {
        swd_target_resume(instance);
    }
    swd_gdb_write(client, "+", (uint32_t)strlen("+"));
}

/**
 * Parse and handle monitor command from client
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 * @param    cmd       Monitor command string.
 *
 * @returns  None
 */
void gdb_monitor(swd_gdb_client_t *client, swd_t *instance, char *cmd)
{
    char command_str[GDB_MONITOR_CMDSTR_MAX] = { 0 };
    int i = 0;

    // Convert ascii hex into a string
    while (cmd[0] && cmd[1] && (i < sizeof(command_str) - 1))
    {
        command_str[i++] = hexstr_to_uint8(cmd);
        cmd += 2;
    }

    if (!strcmp(command_str, "reset"))
    {
        // We'll defer the reset until we resume execution.  GDB always
        // sets the breakpoints right before continue, so we need
        // to stop on any pending breakpoints after reset.
        gdb_pending_reset = 1;
    }
    else if (!strcmp(command_str, "halt"))
    {
        swd_target_halt(instance);
    }
    else if (!strcmp(command_str, "resume"))
    {
        swd_target_resume(instance);
    }
    else
    {
        // Unsupported command
        gdb_reply(client, "");
        return;
    }

    gdb_reply(client, "OK");
}

/**
 * Parse and handle GDB frontend query command.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 * @param    query     Query command from GDB client.
 *
 * @returns  None
 */
void gdb_query(swd_gdb_client_t *client, swd_t *instance, char *query)
{
    char *arg = query;

    while (*arg)
    {
        if (*arg == ',')
        {
            *arg++ = 0;
            break;
        }

        arg++;
    }

    if (!strcmp(query, "Rcmd"))
    {
        gdb_monitor(client, instance, arg);
        return;
    }

    // Not supported
    gdb_reply(client, "");
}

/**
 * Initial parsing of GDB client command.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 * @param    command   Command string.
 *
 * @returns  0
 */
int gdb_parse_command(swd_gdb_client_t *client, swd_t *instance, uint8_t *command)
{
    switch (command[0])
    {
        case 'P':    // Write general registers
            if (gdb_set_register(client, instance, (char *)&command[1]))
            {
                gdb_reply(client, "E00");
                break;
            }

            gdb_reply(client, "OK");
            break;

        case 'g':   // Read general registers
            gdb_dump_registers(client, instance);
            break;

        case 'M':   // Write memory, M<addr>,<length>:<data hexstr>
            if (gdb_write_memory(client, instance, (char *)&command[1]))
            {
                gdb_reply(client, "E00");
                break;
            }

            gdb_reply(client, "OK");
            break;

        case 'm':   // Read memory, m<addr>,<length>
            gdb_read_memory(client, instance, (char *)&command[1]);
            break;

        case 's':   // single step s[addr]
            swd_target_step(instance);
            gdb_reply(client, "S05");  // Trap Signal
            break;

        case 'c':   // continue c[addr]
            gdb_resume(client, instance);
            break;

        case 'z':   // zX,<addr>,<type> delete breakpoint
            if (gdb_breakpoint_clear(client, instance, (char *)&command[1]))
            {
                gdb_reply(client, "E00");
                break;
            }

            gdb_reply(client, "OK");
            break;

        case 'Z':   // ZX,<addr>,<type> insert breakpoint
            if (gdb_breakpoint_set(client, instance, (char *)&command[1]))
            {
                gdb_reply(client, "E00");
                break;
            }

            gdb_reply(client, "OK");
            break;

        case 'q':
            gdb_query(client, instance, (char *)&command[1]);
            break;

        case '?':
            swd_target_halt(instance);
            gdb_reply(client, "S05");  // Trap
            break;

        default:    // Unhandled command, reply with +$#00
            gdb_reply(client, "");
    }

    return 0;
}


/*
 * Stream handling
 */

/**
 * State machine handling for incoming variable sized frames.  Breaks into
 * contiguous gdb command strings for parsing.
 *
 * @param    client    Client instance pointer.
 * @param    instance  SWD driver instance pointer.
 * @param    data      Raw socket data.
 * @param    len       Size of data buffer.
 *
 * @returns  0
 */
int gdb_stream_process(swd_gdb_client_t *client, swd_t *instance, uint8_t *data, uint16_t len)
{
    while (len)
    {
        // ctrl-c - halt target and return status
        if (*data == 0x03)
        {
            swd_target_halt(instance);
            gdb_reply(client, "S02");  // SIGINT
            gdb_target_state = GDB_TARGET_HALTED;
            gdb_stream_state = GDB_STREAM_STATE_IDLE;
        }

        switch (gdb_stream_state)
        {
            case GDB_STREAM_STATE_IDLE:
                if (*data == GDB_START_CHAR)
                {
                    gdb_stream_state = GDB_STREAM_STATE_DATA;
                    gdb_command_len = 0;
                }
                break;

            case GDB_STREAM_STATE_DATA:
                if (gdb_command_len >= (sizeof(gdb_command) - 1))
                {
                    // Exceeded command buffer, bail out, return -
                    break;
                }

                if (*data == GDB_END_CHAR)
                {
                    gdb_stream_state = GDB_STREAM_STATE_CSUM;
                    gdb_csum_len = 0;
                    break;
                }
                gdb_command[gdb_command_len++] = *data;
                break;

            case GDB_STREAM_STATE_CSUM:
                gdb_csum_len++;
                if (gdb_csum_len >= GDB_CSUM_SIZE)
                {
                    // Per GDB specs; Over TCP connections, the CSUM is redundant and
                    // may be ignored.  Duely ignoring here.

                    gdb_command[gdb_command_len++] = 0;
                    gdb_parse_command(client, instance, gdb_command);

                    gdb_stream_state = GDB_STREAM_STATE_IDLE;
                }
                break;

            case GDB_STREAM_STATE_EXIT:
                return -1;
        }

        len--;
        data++;
    }

    return 0;
}

/**
 * Poll target device for halted state.  If halted, send signal to GDB client
 * indicating trap.
 *
 * @param    instance  SWD driver instance pointer.
 * @param    client    Client instance pointer.
 *
 * @returns  None
 */
void gdb_poll(swd_gdb_client_t *client, swd_t *instance)
{
    if (gdb_target_state == GDB_TARGET_RUNNING)
    {
        if (swd_target_is_halted(instance) > 0)
        {
            gdb_target_state = GDB_TARGET_HALTED;
            gdb_reply(client, "S05");  // Trap
        }
    }
}

/**
 * Halt target on initial connection.
 *
 * @param    instance  SWD driver instance pointer.
 *
 * @returns  None
 */
void gdb_halt(swd_t *instance)
{
    swd_target_halt(instance);
}

