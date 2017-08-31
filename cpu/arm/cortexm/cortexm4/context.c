/*
 * sam4_context.c
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
#include "vectors.h"

#include "workqueue.h"


typedef struct
{
    uint32_t r4_11[8];            /* Softwware saved context */
    uint32_t r0;                  /* This line and below start hardware context */
    uint32_t rs[4];
    uint32_t lr;
    uint32_t pc;
    uint32_t cpsr;
} __attribute__ ((packed)) frame_t;


void console_print(char *format, ...);


thread_t main_thread =
{
    .name = "main",
    .state = THREAD_STATE_ACTIVE,
};

thread_list_t thread_active =
{
    .next = &thread_active,
    .prev = &thread_active,
};

thread_list_t thread_pending =
{
    .next = &thread_pending,
    .prev = &thread_pending,
};

thread_t *thread_current = &main_thread;
thread_t *thread_last;


static inline void *stack_ptr_get(void)
{
    void *sp;

    asm volatile ("mrs %0, psp;"
                  : "=r"(sp)        /* output register %0 */
                  :                 /* no input */
                  :                 /* no clobbered register */
            );

    return sp;
}

static inline void stack_ptr_set(void *sp)
{
    asm volatile ("msr psp, %0;"
                  :                 /* no output */
                  : "r"(sp)         /* stack pointer input */
                  :                 /* no clobbered register */
            );
}

static inline void context_save(void)
{
    void *sp;

    /* Push everything onto the current stack */
    asm volatile ("mrs r3, psp;"
                  "stmdb r3!, {r4-r11};" /* r0-3 and r12 are saved by hardware */
                  "msr psp, r3;"
                  : "=r"(sp)        /* output register %0 */
                  :                 /* no input */
                  : "r3"            /* clobbered register */
            );
}

static inline void context_restore(void)
{
    void *sp;

    /* Restore a previous stack, stack pointer must have already been set */
    asm volatile ("mrs r3, psp;"
                  "ldmia r3!, {r4-r11};" /* r0-3 and r12 are restored by hardware */
                  "msr psp, r3;"
                  : "=r"(sp)        /* output register %0 */
                  :                 /* no input */
                  : "r3"            /* clobbered register */
            );
}


static void thread_list_unlink(thread_list_t *thread)
{
    if (!thread->prev || !thread->next)
    {
        return;
    }

    thread->prev->next = thread->next;
    thread->next->prev = thread->prev;

    thread->next = NULL;
    thread->prev = NULL;
}

static void thread_list_append(thread_list_t *head, thread_list_t *entry)
{
    if (entry->prev || entry->next)
    {
        thread_list_unlink(entry);
    }

    entry->next = head;
    entry->prev = head->prev;

    head->prev->next = entry;
    head->prev = entry;
}


void thread_waiter_queue(thread_list_t *head, thread_t *thread)
{
    thread_list_append(head, &thread->wait_list);
}

thread_t *thread_waiter_dequeue(thread_list_t *head)
{
    thread_t *thread;

    if (head->next == head)
    {
        return NULL;
    }

    thread = thread_from_wait_list(head->next);

    thread_list_unlink(head->next);

    return thread;
}

void thread_waiter_remove(thread_t *thread)
{
    thread_list_unlink(&thread->wait_list);
}


void thread_schedule(thread_t *thread)
{
    uint32_t irqstate = irq_save();

    thread->state = THREAD_STATE_ACTIVE;
    thread_list_append(&thread_active, &thread->state_list);

    irq_restore(irqstate);
}

void thread_pend(thread_t *thread, uint32_t state)
{
    uint32_t irqstate = irq_save();

    thread->state = state;

    thread_list_append(&thread_pending, &thread->state_list);

    irq_restore(irqstate);
}

__attribute__ ((noinline)) static void thread_switch_next(void)
{
    thread_list_t *next;

    thread_last = thread_current;

    thread_current->stack = stack_ptr_get();

    // Queue the previously active thread appropriately.
    switch (thread_current->state)
    {
        case THREAD_STATE_ACTIVE:
            thread_list_append(&thread_active, &thread_current->state_list);
            break;

        case THREAD_STATE_PENDING:
            thread_list_append(&thread_pending, &thread_current->state_list);
            break;

        case THREAD_STATE_INACTIVE:
            break;
    }

    // Get the next active thread
    next = thread_active.next;
    if (next == &thread_active)
    {
        console_print("Threading Exception, no active thread,\r\n");
        console_print("idle/workqueue thread may be blocking.\r\n");
        // There should always be a active idle thread, however if not
        // just keep using the current thread.
        return;
    }

    thread_list_unlink(next);

    thread_current = thread_from_state_list(next);

    stack_ptr_set(thread_current->stack);
}

/* Registered as the svcall handler, interrupt context */
void thread_switch_handler(void)
{
    context_save();
    thread_switch_next();
    context_restore();
}


void thread_exit(void)
{
    uint32_t irqstate = irq_save();

    thread_current->state = THREAD_STATE_INACTIVE;
    thread_list_unlink(&thread_current->wait_list);

    irq_restore(irqstate);

    thread_switch();
}

void thread_new(thread_t *thread, void *stack, uint32_t stack_size,
                void (*start_function)(void *), void *arg)
{
    frame_t *new_stack = (frame_t *)((uint8_t *)stack + stack_size -
                                     sizeof(frame_t));

    new_stack->r0 = (uint32_t)arg;
    new_stack->pc = (uint32_t)start_function;
    new_stack->lr = (uint32_t)thread_exit;
    new_stack->cpsr = 0x21000000;

    thread->stack = (uint8_t *)new_stack;

    thread_schedule(thread);
}

void thread_remove(thread_t *thread)
{
    uint32_t irqstate = irq_save();

    thread->state = THREAD_STATE_INACTIVE;
    thread_list_unlink(&thread->state_list);
    thread_list_unlink(&thread->wait_list);

    irq_restore(irqstate);
}


#define THREAD_WORKQUEUE_STACK_SIZE              2048
uint8_t thread_workqueue_stack[THREAD_WORKQUEUE_STACK_SIZE];
thread_t thread_workqueue =
{
    .name = "workqueue/idle",
};
static void thread_workqueue_loop(void *arg)
{
    while (1)
    {
        while (workqueue_handle_next())
            ;

        if (thread_active.next == &thread_active)
        {
            cpu_sleep();
        }

        thread_switch();
    }
}

void thread_init(void)
{
    thread_new(&thread_workqueue, thread_workqueue_stack, sizeof(thread_workqueue_stack),
               thread_workqueue_loop, NULL);
}


#ifdef IMPLEMENT_IN_COMMON
int cmd_threads(uart_drv_t *uart, int argc, char *argv[])
{
    const char *states[] =
    {
        [THREAD_STATE_INACTIVE] = "Inactive",
        [THREAD_STATE_ACTIVE] = "Active",
        [THREAD_STATE_PENDING] = "Pending",
    };
    thread_list_t *heads[] =
    {
        &thread_active,
        &thread_pending
    };
    int i;

    // Header
    console_print("\n%-12s %-8s %-8s %-8s %s\r\n\n",
                  "State", "Thread *", "SP", "LR", "Name");

    // Running is not on any list and is always printed first
    console_print("%-12s %08x %08x %-8s %s\r\n", "Running",
                  (uint32_t)thread_current, (uint32_t)thread_current->stack, "N/A",
                  thread_current->name ? thread_current->name : "N/A");

    // Loop through active, pending lists and show tasks
    for (i = 0; i < (sizeof(heads) / sizeof(heads[0])); i++)
    {
        thread_list_t *next = heads[i]->next;
        while (next != heads[i])
        {
            thread_t *thread = thread_from_state_list(next);
            frame_t *stack = (frame_t *)thread->stack;

            console_print("%-12s %08x %08x %08x %s\r\n", states[thread->state],
                          (uint32_t)thread, (uint32_t)thread->stack,
                          stack->lr & 0xfffffffe,
                          thread->name ? thread->name : "N/A");

            next = next->next;
        }
    }
    console_print("\n");

    return 0;
}
#endif

