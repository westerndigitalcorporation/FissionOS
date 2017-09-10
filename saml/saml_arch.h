/*
 * arch.h
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

#ifndef __ARCH_H__
#define __ARCH_H__


static inline void write32(volatile uint32_t *addr, uint32_t data)
{
    uint32_t tmp = (uint32_t)addr;
    asm volatile ("str %0, [%1];"
                  :
                  : "r"(data), "r"(tmp)
                  : "r3", "r2"
                 );
}

static inline void write16(volatile uint16_t *addr, uint16_t data)
{
    uint32_t tmp = (uint32_t)addr;
    asm volatile ("mov r3, %1; movs r2, %0; strh r2, [r3];"
                  :
                  : "r"(data), "r"(tmp)
                  : "r3", "r2"
                 );
}

static inline void write8(volatile uint8_t *addr, uint8_t data)
{
    uint32_t tmp = (uint32_t)addr;
    asm volatile ("mov r3, %1; mov r2, %0; strb r2, [r3];"
                  :
                  : "r"(data), "r"(tmp)
                  : "r3", "r2"
                 );
}

#endif /* __ARCH_H__ */
