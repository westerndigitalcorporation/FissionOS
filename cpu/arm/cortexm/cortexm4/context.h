/*
 * sam4_contetx.h
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


#ifndef __CONTEXT_H__
#define __CONTEXT_H__


#include "vectors.h"

#ifndef conatinerof
#define containerof(itemptr, type, member)       ((type *)((uint32_t)itemptr - offsetof(type, member)))
#endif
#define thread_from_wait_list(ptr)               containerof(ptr, thread_t, wait_list)
#define thread_from_state_list(ptr)              containerof(ptr, thread_t, state_list)


typedef struct thread_list
{
    struct thread_list *next;
    struct thread_list *prev;
} thread_list_t;

typedef struct thread
{
    thread_list_t state_list;                    /**< State list (Active/Pending) */
    thread_list_t wait_list;                     /**< Wait list, used for semaphores, places
                                                      that the task might be pending */
    void *stack;                                 /**< Context stack pointer */
    uint32_t state;                              /**< Context state (Active/Pending/Etc) */
#define THREAD_STATE_INACTIVE                    0
#define THREAD_STATE_ACTIVE                      1
#define THREAD_STATE_PENDING                     2
    char *name;                                  /**< Name string used for debug/console */
} thread_t;


/** Thread currently running */
extern thread_t *thread_current;


static inline void thread_switch(void)
{
    syscall();
}


void thread_new(thread_t *thread, void *stack, uint32_t stack_size,
                void (*start_function)(void *), void *arg);
void thread_remove(thread_t *thread);

void thread_schedule(thread_t *thread);
void thread_pend(thread_t *thread, uint32_t state);
static inline void thread_pend_current(void)
{
    thread_pend(thread_current, THREAD_STATE_PENDING);
    thread_switch();
}

/** Thread waiting lists, used for semaphores, etc.. */
void thread_waiter_queue(thread_list_t *head, thread_t *thread);
thread_t *thread_waiter_dequeue(thread_list_t *head);
void thread_waiter_remove(thread_t *thread);


void thread_init(void);


#endif /* __CONTEXT_H__ */
