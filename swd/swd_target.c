/*
 * swd_target.c
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

#include "swd_target.h"
#include "swd_chipid.h"

#include "cortexm_debug.h"


/**
 * Halt the CPU.
 *
 * @param    instance   SWD Driver Instance pointer.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_halt(swd_t *instance)
{
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    ret = SWD_CORTEX_HALT(instance);

    swd_stop(instance);

    return ret;
}

/**
 * Check to see if the CPU is currently halted.
 *
 * @param    instance   SWD Driver Instance pointer.
 *
 * Returns   1 if halted, 0 if running, or negative SWD error number.
 */
int swd_target_is_halted(swd_t *instance)
{
    uint32_t dhcsr;
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    ret = swd_mem_read(instance, CORTEX_DHCSR, &dhcsr);
    if (ret)
    {
        // Will be nagative on error condition
        goto done;
    }

    if (dhcsr & CORTEX_DHCSR_HALT)
    {
        ret = 1;
        goto done;
    }

    ret = 0;

done:
    swd_stop(instance);

    return ret;
}

/**
 * Resume the CPU.
 *
 * @param    instance   SWD Driver Instance pointer.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_resume(swd_t *instance)
{
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    ret = SWD_CORTEX_RESUME(instance);

    swd_stop(instance);

    return ret;
}

/**
 * Single step the CPU by one assembly instruction.
 *
 * @param    instance   SWD Driver Instance pointer.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_step(swd_t *instance)
{
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    ret = SWD_CORTEX_STEP(instance);

    swd_stop(instance);

    return ret;
}

/**
 * Reset the CPU (and associated peripherals if implemented by vendor).
 *
 * @param    instance   SWD Driver Instance pointer.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_reset(swd_t *instance)
{
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    ret = SWD_CORTEX_RESET(instance);

    swd_stop(instance);

    return ret;
}

/**
 * Write a byte of data at the address given by addr.
 *
 * @param    instance   SWD Driver Instance pointer.
 * @param    addr       Address for write.
 * @param    data       Data to write.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_write_uint8(swd_t *instance, uint32_t addr, uint8_t data)
{
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    ret = swd_mem_write(instance, (uint32_t)addr & ~0x3, data);

    swd_stop(instance);

    return ret;
}

/**
 * Read a byte of data from the address given by addr.
 *
 * @param    instance   SWD Driver Instance pointer.
 * @param    addr       Address for read.
 * @param    data       Pointer for data to read.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_read_uint8(swd_t *instance, uint32_t addr, uint8_t *data)
{
    uint8_t tmpbuf[sizeof(uint32_t)];
    uint32_t *tmp = (uint32_t *)tmpbuf;
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    ret = swd_mem_read(instance, (uint32_t)addr & ~0x3, tmp);
    swd_stop(instance);

    *data = tmpbuf[addr & 0x3];

    return ret;
}

/**
 * Read the CPU registers into a array of uint32.  The provided array must
 * be sizeof(uint32_t) * CORTEX_MAX_REGS.  Note: CPU must be halted or the
 * results will be unpredictable.
 *
 * @param    instance   SWD Driver Instance pointer.
 * @param    regbuf     Pointer to register array.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_regs(swd_t *instance, uint32_t *regbuf)
{
    int ret, i;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    for (i = 0; i < CORTEX_MAX_REGS; i++)
    {
        ret = swd_mem_write(instance, (uint32_t)CORTEX_DCRSR, CORTEX_DCRSR_REGSEL(i));
        if (ret)
        {
            goto done;
        }

        ret = swd_mem_read(instance, (uint32_t)CORTEX_DCRDR, &regbuf[i]);
        if (ret)
        {
            goto done;
        }
    }

done:
    swd_stop(instance);

    return ret;
}

/**
 * Change CPU register.
 *
 * @param    instance   SWD Driver Instance pointer.
 * @param    reg        Register index.
 * @param    value      Value to set.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_set_reg(swd_t *instance, uint8_t reg, uint32_t value)
{
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    ret = swd_mem_write(instance, (uint32_t)CORTEX_DCRSR,
                        CORTEX_DCRSR_REGSEL(reg) | CORTEX_DCRSR_WRITE);
    if (ret)
    {
        goto done;
    }

    ret = swd_mem_write(instance, (uint32_t)CORTEX_DCRDR, value);
    if (ret)
    {
        goto done;
    }

done:
    swd_stop(instance);

    return ret;
}

/**
 * Set hardware breakpoint.  Caller must ensure that invalid breakpoint
 * numbers aren't used.
 *
 * @param    instance   SWD Driver Instance pointer.
 * @param    bpnum      Breakpointer number.
 * @param    addr       Address on which execution will trigger break.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_breakpoint_set(swd_t *instance, int bpnum, uint32_t addr)
{
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    SWD_CORTEX_BP_SET(instance, bpnum, addr,
                      addr & 0x2 ? CORTEX_FP_COMP_REPLACE_BKPT_UPPER :
                                   CORTEX_FP_COMP_REPLACE_BKPT_LOWER);

    SWD_CORTEX_BP_ENABLE(instance);

    swd_stop(instance);

    return ret;
}

/**
 * Clear hardware breakpoint.  Caller must ensure that invalid breakpoint
 * numbers aren't used.
 *
 * @param    instance   SWD Driver Instance pointer.
 * @param    bpnum      Breakpointer number.
 *
 * Returns   0 or negative SWD error number.
 */
int swd_target_breakpoint_clear(swd_t *instance, int bpnum)
{
    int ret;

    ret = swd_start(instance);
    if (ret)
    {
        return ret;
    }

    SWD_CORTEX_BP_CLEAR(instance, bpnum);

    swd_stop(instance);

    return ret;
}

