/*
 * imx6_memtest.c
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
#include <stdlib.h>


#define DDR3_BASE_ADDR                           0x10000000
#define DDR3_SIZE                                0x10000000


extern uint32_t __bss_start, __bss_end, __stack;
extern uint32_t __memtest_state, __memtest_addr, __memtest_passes;


/*
 *
 * The following functions are taken from the public domain at:
 *
 * http://www.barrgroup.com/Embedded-Systems/How-To/Memory-Test-Suite-C
 *
 * Barr, Michael. "Software-Based Memory Testing," Embedded Systems
 * Programming, July 2000, pp. 28-40.
 *
 */

/*
 * Description: Test the integrity of a physical memory device by
 *              performing an increment/decrement test over the
 *              entire region.  In the process every storage bit
 *              in the device is tested as a zero and a one.  The
 *              base address and the size of the region are
 *              selected by the caller.
 *
 * Returns:     NULL if the test succeeds.
 *
 *              A non-zero result is the first address at which an
 *              incorrect value was read back.  By examining the
 *              contents of memory, it may be possible to gather
 *              additional information about the problem.
 */
uint8_t *memtest_device(volatile uint8_t *base_address, unsigned long n_bytes)
{
    unsigned long offset;
    unsigned long n_words = n_bytes / sizeof(uint8_t);
    uint8_t pattern;
    uint8_t antipattern;

    /*
     * Fill memory with a known pattern.
     */
    for (pattern = 1, offset = 0; offset < n_words; pattern++, offset++)
    {
        base_address[offset] = pattern;
    }

    /*
     * Check each location and invert it for the second pass.
     */
    for (pattern = 1, offset = 0; offset < n_words; pattern++, offset++)
    {
        if (base_address[offset] != pattern)
        {
            return (uint8_t *)&base_address[offset];
        }

        antipattern = ~pattern;
        base_address[offset] = antipattern;
    }

    /*
     * Check each location for the inverted pattern and zero it.
     */
    for (pattern = 1, offset = 0; offset < n_words; pattern++, offset++)
    {
        antipattern = ~pattern;
        if (base_address[offset] != antipattern)
        {
            return (uint8_t *)&base_address[offset];
        }
    }

    return NULL;
}

/**
 * Description: Test the address bus wiring in a memory region by
 *              performing a walking 1's test on the relevant bits
 *              of the address and checking for aliasing. This test
 *              will find single-bit address failures such as stuck
 *              -high, stuck-low, and shorted pins.  The base address
 *              and size of the region are selected by the caller.
 *
 * Notes:       For best results, the selected base address should
 *              have enough LSB 0's to guarantee single address bit
 *              changes.  For example, to test a 64-Kbyte region,
 *              select a base address on a 64-Kbyte boundary.  Also,
 *              select the region size as a power-of-two--if at all
 *              possible.
 *
 * Returns:     NULL if the test succeeds.
 *              A non-zero result is the first address at which an
 *              aliasing problem was uncovered.  By examining the
 *              contents of memory, it may be possible to gather
 *              additional information about the problem.
 */
uint8_t *memtest_address_bus(volatile uint8_t *base_address, unsigned long n_bytes)
{
    uint32_t address_mask = (n_bytes/sizeof(uint8_t) - 1);
    uint32_t offset;
    uint32_t test_offset;
    uint8_t pattern     = 0xaa;
    uint8_t antipattern = 0x55;

    /*
     * Write the default pattern at each of the power-of-two offsets.
     */
    for (offset = 1; (offset & address_mask) != 0; offset <<= 1)
    {
        base_address[offset] = pattern;
    }

    /*
     * Check for address bits stuck high.
     */
    test_offset = 0;
    base_address[test_offset] = antipattern;

    for (offset = 1; (offset & address_mask) != 0; offset <<= 1)
    {
        if (base_address[offset] != pattern)
        {
            return (uint8_t *)&base_address[offset];
        }
    }

    base_address[test_offset] = pattern;

    /*
     * Check for address bits stuck low or shorted.
     */
    for (test_offset = 1; (test_offset & address_mask) != 0; test_offset <<= 1)
    {
        base_address[test_offset] = antipattern;

        if (base_address[0] != pattern)
        {
            return (uint8_t *)&base_address[test_offset];
        }

        for (offset = 1; (offset & address_mask) != 0; offset <<= 1)
        {
            if ((base_address[offset] != pattern) && (offset != test_offset))
            {
                return (uint8_t *)&base_address[test_offset];
            }
        }

        base_address[test_offset] = pattern;
    }

    return NULL;
}

/**
 * Description: Test the data bus wiring in a memory region by
 *              performing a walking 1's test at a fixed address
 *              within that region.  The address (and hence the
 *              memory region) is selected by the caller.
 *
 * Returns:     0 if the test succeeds.
 *              A non-zero result is the first pattern that failed.
 */
uint32_t memtest_data_bus(volatile uint32_t *address)
{
    uint32_t pattern;

    /*
     * Perform a walking 1's test at the given address.
     */
    for (pattern = 1; pattern != 0; pattern <<= 1)
    {
        /*
         * Write the test pattern.
         */
        *address = pattern;

        /*
         * Read it back (immediately is okay for this test).
         */
        if (*address != pattern)
        {
            return pattern;
        }
    }

    return 0;
}

void memtest(void)
{
    __memtest_state = 0;
    __memtest_passes = 0;

    while (1)
    {
        __memtest_addr = 0;
        if ((__memtest_state = (uint32_t)memtest_data_bus((uint32_t *)DDR3_BASE_ADDR)))
        {
            break;
        }

        __memtest_addr = 1;
        if ((__memtest_state = (uint32_t)memtest_address_bus((uint8_t *)DDR3_BASE_ADDR, DDR3_SIZE)))
        {
            break;
        }

        __memtest_addr = 2;
        if ((__memtest_state = (uint32_t)memtest_device((uint8_t *)DDR3_BASE_ADDR, DDR3_SIZE)))
        {
            break;
        }

        __memtest_passes++;
    }
}

int main(void)
{
    uint32_t *start = &__bss_start;

    // Clear out the .bss
    while (start < &__bss_end)
    {
        *start++ = 0;
    }

    memtest();

    while (1)
        ;

    return 0;
}
