/*
 * sys_arch.c
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
#include <stddef.h>
#include <string.h>

#include "cc.h"
#include "sys_arch.h"
#include "lwip/sys.h"


err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
    memset(sem, 0, sizeof(*sem));
    sem_init(sem, count);
    return ERR_OK;
}

int sys_sem_valid(sys_sem_t *sem)
{
    return (sem->flags & SEM_FLAGS_INVALID) ? 0 : 1;
}

void sys_sem_set_invalid(sys_sem_t *sem)
{
    sem->flags |= SEM_FLAGS_INVALID;
}

void sys_sem_free(sys_sem_t *sem)
{
    sem->flags |= SEM_FLAGS_INVALID;
}

void sys_sem_signal(sys_sem_t *sem)
{
    sem_post(sem);
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    uint32_t start = sys_now();

    if (sem_take(sem, timeout))
    {
        return SYS_ARCH_TIMEOUT;
    }

    return sys_now() - start;
}

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
    memset(mbox, 0, sizeof(*mbox));
    return ERR_OK;
}

int sys_mbox_valid(sys_mbox_t *mbox)
{
    return (mbox->flags & MAILBOX_FLAGS_INVALID) ? 0 : 1;
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
    mbox->flags |= MAILBOX_FLAGS_INVALID;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
    mbox->flags |= MAILBOX_FLAGS_INVALID;
}

void sys_mbox_post(sys_mbox_t *mbox, void *data)
{
    while (mailbox_send(mbox, data) == -1)
    {
        thread_switch();
    }
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *data)
{
    if (mailbox_send(mbox, data))
    {
        return ERR_MEM;
    }

    return ERR_OK;
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **data, u32_t timeout)
{
    uint32_t start = sys_now();

    if (mailbox_recv(mbox, data, timeout))
    {
        return SYS_ARCH_TIMEOUT;
    }

    return sys_now() - start;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **data)
{
    if (mailbox_tryrecv(mbox, data))
    {
        return SYS_MBOX_EMPTY;
    }

    return 0;
}

uint8_t stack_pool[SYS_ARCH_STACK_POOL_SIZE] THREAD_STACK;
uint32_t stack_pool_offset = 0;
sys_arch_thread_t lwip_threads[SYS_ARCH_THREADS_MAX];
sys_thread_t sys_thread_new(const char *name, void (*thread)(void *arg),
                            void *arg, int stacksize, int prio)
{
    int i;

    if (stacksize > sizeof(stack_pool) - stack_pool_offset)
    {
        return NULL;
    }

    for (i = 0; i < SYS_ARCH_THREADS_MAX; i++)
    {
        if (!(lwip_threads[i].flags & SYS_THREAD_FLAGS_ALLOC))
        {
            lwip_threads[i].flags |= SYS_THREAD_FLAGS_ALLOC;
            lwip_threads[i].thread.name = (char *)name;
            thread_new(&lwip_threads[i].thread, &stack_pool[stack_pool_offset],
                       stacksize, thread, arg);
            stack_pool_offset += stacksize;

            return &lwip_threads[i];
        }
    }

    return NULL;
}

void sys_init(void)
{
}

