/*
 * cc.h
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


#ifndef __CC_H__
#define __CC_H__

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#include <errno.h>

#include <console.h>

#define U16_F "d"
#define S16_F "d"
#define X16_F "x"
#define U32_F "d"
#define S32_F "d"
#define X32_F "x"

#define PACK_STRUCT_FIELD(x)                     x
#define PACK_STRUCT_STRUCT                       __attribute__ ((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

#define BYTE_ORDER                               LITTLE_ENDIAN

#define LWIP_PLATFORM_DIAG(y)                    console_message_writef y

#define LWIP_PLATFORM_ASSERT(y)                  console_assert(y)

#define LWIP_RAND()                              rand()

#define SYS_ARCH_DECL_PROTECT(x)                 volatile uint32_t x
#define SYS_ARCH_PROTECT(x)		                 cli_save_sreg(&x)
#define SYS_ARCH_UNPROTECT(x)		             restore_sreg(&x)


typedef uint8_t u8_t;
typedef int8_t s8_t;
typedef uint16_t u16_t;
typedef int16_t s16_t;
typedef uint32_t u32_t;
typedef int32_t s32_t;
typedef int mem_ptr_t;

#endif /* __CC_H__ */
