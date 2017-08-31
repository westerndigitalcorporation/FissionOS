/*
 * sam4_inputchange.c
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

#include "vectors.h"

#include "sam4_clock.h"
#include "sam4_gpio.h"
#include "sam4_inputchange.h"

// Global array and count, populated on registration.
input_change_t *input_change_array;
uint8_t input_change_array_size;

static void input_change_pioa(void);
static void input_change_piob(void);
static void input_change_pioc(void);

//
// @brief   Setup the callbacks and enable the interrupts for the input
//          change events.
//
// @returns None
//
static void input_change_setup_ints(void)
{
    nvic_callback_set(PERIPHERAL_ID_PIOA, input_change_pioa);
    nvic_enable(PERIPHERAL_ID_PIOA);
    nvic_callback_set(PERIPHERAL_ID_PIOB, input_change_piob);
    nvic_enable(PERIPHERAL_ID_PIOB);
    nvic_callback_set(PERIPHERAL_ID_PIOC, input_change_pioc);
    nvic_enable(PERIPHERAL_ID_PIOC);
}

//
// @brief   Register the input change interrupt handlers and setup the
//          hardware.
//
// @param   handlers      Array of input change configs and handlers.
// @param   num_elements  Number of items in the array.
//
// @returns None
//
void input_change_init(input_change_t *handlers, uint8_t num_elements)
{
    uint8_t i;

    input_change_setup_ints();

    // Setup the globals
    input_change_array = handlers;
    input_change_array_size = num_elements;

    // Configure the hardware
    for (i = 0; i < input_change_array_size; i++)
    {
        input_change_t       *input = &input_change_array[i];
        volatile gpio_regs_t *port  = input->port;

        // Subsequent calls will have no effect if the clock is already on
        if (input->port == GPIOA)
        {
            clock_peripheral_start(PERIPHERAL_ID_PIOA);
        } else if (input->port == GPIOB)
        {
            clock_peripheral_start(PERIPHERAL_ID_PIOB);
        } else if (input->port == GPIOC)
        {
            clock_peripheral_start(PERIPHERAL_ID_PIOC);
        }

        port->ier = input->mask;
        if (input->callback)
        {
            input->callback(input);
        }

        input->last_state = input->port->pdsr & input->mask;
    }
}

//
// Interrupt handling follows
//

//
// @brief   Common handler for all input change interrupt sources.
//          Locates and finds the proper entry based on port states,
//          and calls proper handler.
//
// @param   port   AVR Port from which the interrupt was signalled.
//
// @returns None
//
static void input_change_handle_int(volatile gpio_regs_t *port)
{
    uint8_t i;

    // Match up the port with the input change entry and issue
    // its callback.
    for (i = 0; i < input_change_array_size; i++)
    {
        input_change_t *input = &input_change_array[i];

        if (input->port == port)
        {
            uint8_t j;

            for (j = 0; j < (sizeof(input->mask) * 8); j++)  // Check all bits
            {
                if ((input->mask & (1 << j)) && input->callback)
                {
                    uint32_t state = input->port->pdsr & (1 << j);

                    // Only trigger callback if current_state != last_state
                    if (input->last_state != state)
                    {
                        input->callback(input);
                        input->last_state = state;
                    }

                    break;
                }
            }
        }
    }
}

//
// Actual interrupt handlers, will call common handler code above.
//
static void input_change_pioa(void)
{
    // Clear the interrupt
    volatile uint32_t data = GPIOA->isr;
    (void)data;

    input_change_handle_int(GPIOA);
}

static void input_change_piob(void)
{
    // Clear the interrupt
    volatile uint32_t data = GPIOB->isr;
    (void)data;

    input_change_handle_int(GPIOB);
}

static void input_change_pioc(void)
{
    // Clear the interrupt
    volatile uint32_t data = GPIOC->isr;
    (void)data;

    input_change_handle_int(GPIOC);
}

