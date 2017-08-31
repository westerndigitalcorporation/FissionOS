/*
 * sys_arch.h
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


#ifndef __SYS_ARCH_H__
#define __SYS_ARCH_H__

#include "context.h"

#include "semaphore.h"
#include "mailbox.h"

#define SYS_ARCH_STACK_POOL_SIZE                 TCPIP_THREAD_STACKSIZE
#define SYS_ARCH_THREADS_MAX                     2
typedef struct
{
    thread_t thread;
    uint32_t flags;
#define SYS_THREAD_FLAGS_ALLOC                   0x1
} sys_arch_thread_t;

#define sys_sem_t                                sem_t
#define sys_mbox_t                               mailbox_t
#define sys_thread_t                             sys_arch_thread_t *

#endif /* __SYS_ARCH_H__ */
