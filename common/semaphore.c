/*
 * semaphore.c
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

#include "context.h"
#include "systick.h"
#include "vectors.h"

#include "semaphore.h"


#define DECLARE_SEM_LOCAL(name, s, wt)           \
    sem_local_t name =                           \
    {                                            \
        .sem = s,                                \
        .waiting_thread = wt,                    \
        .flags = 0,                              \
    }

typedef struct sem_local
{
    sem_t *sem;
    thread_t *waiting_thread;
    volatile uint32_t flags;
#define SEM_FLAGS_TIMEOUT                        0x01
} sem_local_t;


static void semaphore_worker(void *arg)
{
    sem_local_t *local = (sem_local_t *)arg;
    uint32_t state = irq_save();

    thread_waiter_remove(local->waiting_thread);

    local->flags |= SEM_FLAGS_TIMEOUT;
    thread_schedule(local->waiting_thread);

    irq_restore(state);
}

void sem_init(sem_t *sem, uint32_t count)
{
    sem->waiter_list.next = &sem->waiter_list;
    sem->waiter_list.prev = &sem->waiter_list;
    sem->count = count;
}

int sem_post(sem_t *sem)
{
    uint32_t irqstate = irq_save();
    thread_t *thread;

    sem->count++;

    thread = thread_waiter_dequeue(&sem->waiter_list);
    if (thread)
    {
        thread_schedule(thread);
    }

    irq_restore(irqstate);

    return 0;
}

int sem_try(sem_t *sem)
{
    uint32_t irqstate = irq_save();

    if (!sem->count)
    {
        irq_restore(irqstate);
        return -1;
    }

    sem->count--;

    irq_restore(irqstate);
    return 0;
}

int sem_take(sem_t *sem, uint32_t timeout_ms)
{
    uint32_t state = irq_save();
    while (sem_try(sem))
    {
        DECLARE_SEM_LOCAL(local, sem, thread_current);
        DECLARE_WORKQUEUE(wq, semaphore_worker, &local);

        if (timeout_ms)
        {
            workqueue_add(&wq, (timeout_ms * SYSTICK_FREQ) / 1000);
        }

        thread_waiter_queue(&sem->waiter_list, thread_current);

        thread_pend(thread_current, THREAD_STATE_PENDING);
        irq_restore(state);
        thread_switch();
        state = irq_save();

        if (local.flags & SEM_FLAGS_TIMEOUT)
        {
            irq_restore(state);
            return -1;
        }

        workqueue_remove(&wq);
    }

    irq_restore(state);
    return 0;
}

