/*
 * workqueue.c
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

#ifndef WORKQUEUE_TEST

#include "vectors.h"
#include "systick.h"

#else

#define irq_save()            0
#define irq_restore(arg)      do { (void)arg; } while(0)
uint32_t ticks = 0;

#endif

#include "workqueue.h"

workqueue_t workqueue_head =
{
    .next = &workqueue_head,
    .prev = &workqueue_head,
};

static void workqueue_insert_after(workqueue_t *current, workqueue_t *entry)
{
    entry->next = current->next;
    entry->prev = current;

    current->next->prev = entry;
    current->next = entry;
}

static void workqueue_unlink(workqueue_t *wq)
{
    if (!wq->prev || !wq->next)
    {
        return;
    }

    wq->prev->next = wq->next;
    wq->next->prev = wq->prev;

    wq->next = NULL;
    wq->prev = NULL;
}

void workqueue_remove(workqueue_t *wq)
{
    uint32_t irqstate = irq_save();

    workqueue_unlink(wq);

    irq_restore(irqstate);
}

void workqueue_add(workqueue_t *wq, uint32_t wait_ticks)
{
    uint32_t irqstate = irq_save();
    workqueue_t *current;

    workqueue_unlink(wq);

    wq->expire_ticks = ticks + wait_ticks;

    current = workqueue_head.next;
    while (current != &workqueue_head)
    {
        if (current->expire_ticks < wq->expire_ticks)
        {
            break;
        }

        current = current->next;
    }

    workqueue_insert_after(current, wq);

    irq_restore(irqstate);
}

int workqueue_handle_next(void)
{
    uint32_t irqstate = irq_save();
    workqueue_t *current;
    int ret = 0;

    current = workqueue_head.next;
    while (current != &workqueue_head)
    {
        if (current->expire_ticks <= ticks)
        {
            workqueue_unlink(current);
            ret = 1;
            break;
        }

        current = current->next;
    }

    irq_restore(irqstate);

    if (current != &workqueue_head)
    {
        current->callback(current->arg);
    }

    return ret;
}

#ifdef WORKQUEUE_TEST

void wq_test_handler(void *arg)
{
    int *test = (int *)arg;

    if (*test)
    {
        *test = 0;
    }
}

workqueue_t test_wq =
{
    .callback = wq_test_handler,
};

int main(int argc, char *argv[])
{
    int wq_handler_run = 1;
    test_wq.arg = &wq_handler_run;
    workqueue_add(&test_wq, 0);
    workqueue_handle_next();
    return wq_handler_run;
}

#endif



