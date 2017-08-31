/*
 * mailbox.c
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

#include "workqueue.h"

#include "mailbox.h"


static void mailbox_worker(void *arg)
{
    mailbox_t *mb = (mailbox_t *)arg;
    uint32_t state = irq_save();

    mb->flags |= MAILBOX_FLAGS_TIMEOUT;
    thread_schedule(mb->waiting_thread);

    irq_restore(state);
}

int mailbox_send(mailbox_t *mb, void *msg)
{
    uint32_t state = irq_save();
    uint8_t next_head = mb->head + 1;

    if (next_head >= MAILBOX_MSGS_MAX)
    {
        next_head = 0;
    }

    if (next_head == mb->tail)
    {
        irq_restore(state);
        return -1;
    }

    mb->msg[mb->head] = msg;
    mb->head = next_head;

    if (mb->waiting_thread)
    {
        thread_schedule(mb->waiting_thread);
    }

    irq_restore(state);

    return 0;
}

int mailbox_tryrecv(mailbox_t *mb, void **msg)
{
    uint32_t state = irq_save();
    if (mb->tail == mb->head)
    {
        irq_restore(state);
        return -1;
    }

    *msg = mb->msg[mb->tail];

    mb->tail++;
    if (mb->tail >= MAILBOX_MSGS_MAX)
    {
        mb->tail = 0;
    }

    irq_restore(state);
    return 0;
}

int mailbox_recv(mailbox_t *mb, void **msg, uint32_t timeout_ms)
{
    uint32_t state = irq_save();

    while (mailbox_tryrecv(mb, msg))
    {
        DECLARE_WORKQUEUE(wq, mailbox_worker, mb);

        mb->waiting_thread = thread_current;
        mb->flags &= ~MAILBOX_FLAGS_TIMEOUT;

        if (timeout_ms)
        {
            workqueue_add(&wq, (timeout_ms * SYSTICK_FREQ) / 1000);
        }

        // Block for timeout (workqueue) or message (mailbox_send)
        thread_pend(thread_current, THREAD_STATE_PENDING);
        irq_restore(state);
        thread_switch();
        state = irq_save();

        mb->waiting_thread = NULL;

        if (mb->flags & MAILBOX_FLAGS_TIMEOUT)
        {
            irq_restore(state);
            return -1;
        }

        workqueue_remove(&wq);
    }

    irq_restore(state);
    return 0;
}


