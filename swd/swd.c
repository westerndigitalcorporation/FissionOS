/*
 * swd.c
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

#include "swd.h"


/**
 * Calculate the parity on the request bits of a transaction.
 *
 * @param    req    Request bits 0 through 4.
 *
 * @returns  1 or 0 depending on parity calculation.
 */
static int swd_request_parity(uint8_t req)
{
    uint8_t parity = 0;
    int i;

    // Only calculate parity on bits 1-4
    req = (req >> 1) & 0xf;

    for (i = 0; i < 4; i++)
    {
        parity ^= req & 1;
        req >>= 1;
    }

    return parity;
}

/**
 * Given a AP/DP Address, Read or Write, and Address, compose a SWD
 * request command.
 *
 * @param    apdp         AP/DP Destination Address.
 * @param    readwrite    SWD_READ or SWD_WRITE.
 * @param    addr         8-bit AP/DP Register Address.
 *
 * @returns  Request command byte.
 */
static uint8_t swd_build_request(int apdp, int readwrite, uint8_t addr)
{
    uint8_t request = SWD_REQUEST_START | SWD_REQUEST_PARK;

    if (apdp == SWD_AP)
    {
        request |= SWD_REQUEST_AP;
    }

    if (readwrite == SWD_READ)
    {
        request |= SWD_REQUEST_READ;
    }

    request |= SWD_REQUEST_ADDR(addr);

    if (swd_request_parity(request))
    {
        request |= SWD_REQUEST_PARITY;
    }

    return request;
}

/**
 * Read 32-bit register from AP/DP.
 *
 * @param    instance     SWD driver instance pointer.
 * @param    apdp         AP/DP Destination Address.
 * @param    reg          AP/DP Register Address.
 * @param    ack          Pointer to reponse ACK.
 * @param    data         Pointer to returned data.
 *
 * @returns  0 or negative SWD error number.
 */
static int swd_read(swd_t *instance, int apdp, uint8_t reg, uint8_t *ack, uint32_t *data)
{
    uint8_t request = swd_build_request(apdp, SWD_READ, (reg & 0xf) >> 2);
    int result;

    instance->flush(instance);
    result = instance->request_recv(instance, request, ack, data);
    if (result)
    {
        return result;
    }

    return 0;
}

/**
 * Write 32-bit value to AP/DP register.
 *
 * @param    instance     SWD driver instance pointer.
 * @param    apdp         AP/DP Destination Address.
 * @param    reg          AP/DP Register Address.
 * @param    ack          Pointer to reponse ACK.
 * @param    data         Data to write.
 *
 * @returns  0 or negative SWD error number.
 */
static int swd_write(swd_t *instance, int apdp, uint8_t reg, uint8_t *ack, uint32_t data)
{
    uint8_t request = swd_build_request(apdp, SWD_WRITE, (reg & 0xf) >> 2);
    int result;

    instance->flush(instance);
    result = instance->request_xmit(instance, request, ack, data);
    if (result)
    {
        return result;
    }

    return 0;
}

/**
 * Read indirect 32-bit register from AP.
 *
 * @param    instance     SWD driver instance pointer.
 * @param    reg          AP Register Address.
 * @param    ack          Pointer to reponse ACK.
 * @param    data         Pointer to returned data.
 *
 * @returns  0 or negative SWD error number.
 */
static int swd_ap_reg_read(swd_t *instance, uint8_t reg, uint8_t *ack, uint32_t *data)
{
    int result;

    result = swd_write(instance, SWD_DP, SWD_DP_APSELECT, ack,
                       SWD_DP_APSELECT_APBANK(reg));
    if (result)
    {
        return result;
    }

    result = swd_read(instance, SWD_AP, reg, ack, data);
    if (result)
    {
        return result;
    }

    return swd_read(instance, SWD_DP, SWD_DP_READBUFFER, ack, data);
}

/**
 * Write indirect 32-bit value to AP register.
 *
 * @param    instance     SWD driver instance pointer.
 * @param    reg          AP Register Address.
 * @param    ack          Pointer to reponse ACK.
 * @param    data         Data to write.
 *
 * @returns  0 or negative SWD error number.
 */
static int swd_ap_reg_write(swd_t *instance, uint32_t reg, uint8_t *ack, uint32_t data)
{
    int result;

    result = swd_write(instance, SWD_DP, SWD_DP_APSELECT, ack,
                       SWD_DP_APSELECT_APBANK(reg));
    if (result)
    {
        return result;
    }

    result = swd_write(instance, SWD_AP, reg, ack, data);
    if (result)
    {
        return result;
    }

    return 0;
}

/**
 * Send the SWD command to powerup the CPU(s), because they might be sleepy.
 *
 * @param    instance     SWD driver instance pointer.
 *
 * @returns  0 or negative SWD error number.
 */
static int swd_powerup(swd_t *instance)
{
    uint32_t data = 0;
    uint8_t ack;
    int result;

    data = SWD_DP_CTRLSTAT_SYS_POWERUP_REQ | SWD_DP_CTRLSTAT_DBG_POWERUP_REQ;

    // Turn on the cpu power and debug interface if sleeping/disabled
    result = swd_write(instance, SWD_DP, SWD_DP_CTRLSTAT, &ack, data);
    if (result)
    {
        return result;
    }

    while (!result)
    {
        result = swd_read(instance, SWD_DP, SWD_DP_CTRLSTAT, &ack, &data);
        if (data & (SWD_DP_CTRLSTAT_SYS_POWERUP_ACK |
                    SWD_DP_CTRLSTAT_DBG_POWERUP_ACK))
            break;
    }

    return 0;
}

/**
 * Read 32-bit data from the AHB bus through the AP/DP.  On the Cortex-M
 * series CPUs, this is the same address the CPU would use.
 *
 * @param    instance     SWD driver instance pointer.
 * @param    addr         Address of data to read.
 * @param    data         Pointer to return data in.
 *
 * @returns  0 or negative SWD error number.
 */
int swd_mem_read(swd_t *instance, uint32_t addr, uint32_t *data)
{
    uint32_t status;
    uint8_t ack;
    int result;

    // Select and read the AP ID register, required to unlock the AP
    result = swd_ap_reg_write(instance, SWD_AP_TAR, &ack, addr);
    if (result)
    {
        return result;
    }


    result = swd_read(instance, SWD_AP, SWD_AP_DRW, &ack, data);
    if (result)
    {
        return result;
    }


    while (!result)
    {
        result = swd_read(instance, SWD_DP, SWD_DP_CTRLSTAT, &ack, &status);
        if (result)
        {
            return result;
        }

        if (status & SWD_DP_CTRLSTAT_READOK)
        {
            break;
        }
    }

    result = swd_read(instance, SWD_DP, SWD_DP_READBUFFER, &ack, data);
    if (result)
    {
        return result;
    }

    return 0;
}

/**
 * Write 32-bit data through the AHB bus and AP/DP.  On the Cortex-M
 * series CPUs, this is the same address the CPU would use.
 *
 * @param    instance     SWD driver instance pointer.
 * @param    addr         Address of data to write.
 * @param    data         Data to write.
 *
 * @returns  0 or negative SWD error number.
 */
int swd_mem_write(swd_t *instance, uint32_t addr, uint32_t data)
{
    uint8_t ack;
    int result;

    // Select and read the AP ID register, required to unlock the AP
    result = swd_ap_reg_write(instance, SWD_AP_TAR, &ack, addr);
    if (result)
    {
        return result;
    }

    result = swd_write(instance, SWD_AP, SWD_AP_DRW, &ack, data);
    if (result)
    {
        return result;
    }

    return 0;
}

/**
 * Send the SWD startup sequence.  This converts the TMS/TCK of the
 * JTAG to use SWD mode instead.  The IDCODE must be read, and any
 * previous transactions cleared before the bus is ready.
 *
 * @param    instance     SWD driver instance pointer.
 *
 * @returns  0 or negative SWD error number.
 */
int swd_start(swd_t *instance)
{
    uint32_t data;
    uint8_t ack;
    int result;

    if (instance->start)
    {
        result = instance->start(instance);
        if (result)
        {
            return result;
        }
    }

    // Read the IDCODE, required as part of SWD selection
    result = swd_read(instance, SWD_DP, SWD_DP_IDCODE, &ack, &data);
    if (result)
    {
        return result;
    }

    // Abort any previously failed transactions, restores read/write states
    result = swd_write(instance, SWD_DP, SWD_DP_ABORT, &ack, SWD_DP_ABORT_ALL);
    if (result)
    {
        return result;
    }

    result = swd_powerup(instance);
    if (result)
    {
        return result;
    }

    result = swd_ap_reg_read(instance, SWD_AP_IDR, &ack, &data);
    if (result)
    {
        return result;
    }

    result = swd_ap_reg_write(instance, SWD_AP_CSW, &ack,
                              SWD_AP_CSW_DBGSTATUS | SWD_AP_CSW_HPROT1 |
                              SWD_AP_CSW_DBGMASTER | SWD_AP_CSW_32BIT);

    // All your bus is belong to us

    return result;
}

/**
 * Call down to the driver layer to do any port/pin hardware cleanup.
 *
 * @param    instance     SWD driver instance pointer.
 *
 * @returns  0 or negative SWD error number.
 */
int swd_stop(swd_t *instance)
{
    if (instance->stop)
    {
        return instance->stop(instance);
    }

    return 0;
}
