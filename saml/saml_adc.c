/*
 * sam4_adc.c
 *
 *
 * Copyright (c) 2017 Western Digital Corporation or its affiliates.
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


#if defined(__ATSAMD53__)

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

#include <vectors.h>
#include <workqueue.h>

#include "saml_nvm.h"

#include "saml_adc.h"


/* ADC Driver Internal Processing States */
#define ADC_DRV_STATE_IDLE                  0
#define ADC_DRV_STATE_START                 1
#define ADC_DRV_STATE_SAMPLING              2

adc_desc_t *adc_desc[ADC_COUNT];

/**
 * Append a new ADC sample item to the end of the sample queue.
 *
 * @param    entry  ADC sample descriptor pointer.
 *
 * @returns  None
 */
void adc_queue_add(adc_desc_t *desc, adc_queue_entry_t *entry)
{
    volatile uint32_t irq = irq_save();

    // If already on the queue, ignore the add request
    if (entry->next || entry->prev)
    {
        irq_restore(irq);
        return;
    }

    /* link */
    entry->next = (adc_queue_entry_t *)&desc->queue_head;
    entry->prev = desc->queue_head.prev;

    desc->queue_head.prev->next = entry;
    desc->queue_head.prev = entry;

    irq_restore(irq);
}

/**
 * Remove a new ADC sample item from the sample queue.
 *
 * @param    entry  ADC sample descriptor pointer.
 *
 * @returns  None
 */
void adc_queue_remove(adc_queue_entry_t *entry)
{
    volatile uint32_t irq = irq_save();

    // Don't remove something already gone
    if (!entry->next || !entry->prev)
    {
        irq_restore(irq);
        return;
    }

    /* Unlink */
    entry->prev->next = entry->next;
    entry->next->prev = entry->prev;

    /* Mark removed */
    entry->next = NULL;
    entry->prev = NULL;

    irq_restore(irq);
}

/**
 * Return the next ADC queue entry for processing.
 *
 * @returns  ADC queue entry pointer, NULL if None.
 */
static adc_queue_entry_t *adc_queue_first(adc_desc_t *desc)
{
    volatile uint32_t irq = irq_save();
    adc_queue_entry_t *next = desc->queue_head.next;

    if (next == &desc->queue_head)
    {
        next = NULL;
    }

    irq_restore(irq);

    return next;
}

/**
 * Call the descriptor callback function if applicable.
 *
 * @param    adc_drv  ADC driver descriptor pointer.
 *
 * @returns  None
 */
static void adc_do_setup(adc_drv_t *adc_drv)
{
    if (adc_drv->setup)
    {
        adc_drv->setup(adc_drv, adc_drv->setup_arg);
    }
}

/**
 * All conversions are done for a given queue entry.  Go through and
 * call all appropriate calculation callbacks for each conversion in
 * this set.
 *
 * @returns  None
 */
static void adc_do_calc(adc_desc_t *desc)
{
    adc_queue_entry_t *next = adc_queue_first(desc);
    adc_drv_t *adc_drv = next->adcs[desc->active];
    int i;

    desc->conversion_results[0] = desc->adc->result;

    if (adc_drv->calc)
    {
        uint32_t min = desc->conversion_results[0];
        uint32_t max = desc->conversion_results[0];
        uint64_t total = 0;

        for (i = 0; i < ADC_AVERAGE_SAMPLES; i++)
        {
            total += desc->conversion_results[i];

            if (desc->conversion_results[i] < min)
            {
                min = desc->conversion_results[i];
            }

            if (desc->conversion_results[i] > max)
            {
                max = desc->conversion_results[i];
            }
        }

        adc_drv->calc(adc_drv, adc_drv->value, total, min, max,
                      ADC_AVERAGE_SAMPLES, adc_drv->calc_arg);
    }
}

/**
 * Setup the hardware to perform a given ADC conversion.
 *
 * @param    adc_drv  ADC driver descriptor pointer.
 *
 * @returns  None
 */
static void adc_hw_setup(adc_desc_t *desc, adc_drv_t *adc_drv)
{
    volatile adc_t *adc = desc->adc;
    volatile uint32_t irq = irq_save();

    adc_drv->total = 0;

    // Average 64 samples
    adc->avgctrl = ADC_AVGCTRL_SAMPLENUM(0x6) | 
                   ADC_AVGCTRL_ADJRES(0x4);

    // Input
    adc->inputctrl = ADC_INPUTCTRL_MUXPOS(adc_drv->channel);

    irq_restore(irq);
}

/**
 * Start the hardware ADC conversion.
 *
 * @returns  None
 */
static void adc_conversion_start(adc_desc_t *desc)
{
    volatile adc_t *adc = desc->adc;
    volatile uint32_t irq = irq_save();

    desc->state = ADC_DRV_STATE_SAMPLING;

    adc->intenset = ADC_INTENSET_RESRDY;
    adc->swtrig = ADC_SWTRIG_START;

    irq_restore(irq);
}

/**
 * Find and start the next ADC conversion for each channel in the set, and
 * if the set is complete, call the completion callbacks, and start the next
 * element in the queue.
 *
 * @returns  None
 */
static void adc_next(adc_desc_t *desc)
{
    adc_queue_entry_t *entry = adc_queue_first(desc);
    adc_drv_t *adc_drv = entry->adcs[desc->active];

    // If we've done the last one, call the completion routine and go
    // back to idle.
    if (!desc->count)
    {
        // Dequeue
        adc_queue_remove(entry);

        // Execute completions
        if (entry->complete)
        {
            entry->complete(entry->adcs, desc->active, entry->complete_arg);
        }

        // Get next
        entry = adc_queue_first(desc);
        if (!entry)
        {
            // None, stop processing.
            desc->state = ADC_DRV_STATE_IDLE;
            return;
        }

        // Start again
        desc->active = 0;
        desc->count = entry->count;
        adc_drv = entry->adcs[desc->active];
    }

    adc_hw_setup(desc, adc_drv);
    adc_do_setup(adc_drv);

    desc->count--;

    adc_conversion_start(desc);
}

/**
 * Main ADC state machine processing.  This is called from workqueue
 * context.
 *
 * @param    arg    Workqueue context, unused.
 *
 * @returns  None
 */
static void adc_handle_state(void *arg)
{
    adc_desc_t *desc = (adc_desc_t *)arg;
    adc_queue_entry_t *next;

    switch (desc->state)
    {
        case ADC_DRV_STATE_IDLE:
            next = adc_queue_first(desc);
            if (!next)
            {
                break;
            }

            desc->active = 0;
            desc->count = next->count;
            desc->state = ADC_DRV_STATE_START;

        case ADC_DRV_STATE_START:
            adc_next(desc);
            break;

        case ADC_DRV_STATE_SAMPLING:
            adc_do_calc(desc);

            desc->active++;
            adc_next(desc);
            break;
    }
}

/**
 * Handle the ADC interrupt.  ACK and disable interrupt and schedule
 * ADC processing workqueue for processing.
 *
 * @returns  None
 */
static void adc0_handle_interrupt(void)
{
    ADC0->intenclr = ADC_INTENCLR_RESRDY;

    workqueue_add(&adc_desc[0]->wq, 0);
}

/**
 * Handle the ADC interrupt.  ACK and disable interrupt and schedule
 * ADC processing workqueue for processing.
 *
 * @returns  None
 */
static void adc1_handle_interrupt(void)
{
    ADC1->intenclr = ADC_INTENCLR_RESRDY;

    workqueue_add(&adc_desc[1]->wq, 0);
}

/**
 * Query to see if a given ADC queue element has been completed.
 *
 * @params   entry   ADC queue entry pointer.
 *
 * @returns  1 if complete, 0 otherwise.
 */
int adc_finished(adc_queue_entry_t *entry)
{
    return (volatile adc_queue_entry_t *)entry->next ? 0 : 1;
}

/**
 * Initialize the ADC driver.  Setup interrupt handler and turn on device
 *
 * @returns  None
 */
int adc_init(int adcnum, adc_desc_t *desc)
{
    switch (adcnum)
    {
        case 0:
            desc->adc = ADC0;
            break;
        case 1:
            desc->adc = ADC1;
            break;

        default:
            return -1;
    }

    adc_desc[adcnum] = desc;

    desc->state = ADC_DRV_STATE_IDLE;
    desc->queue_head.next = &desc->queue_head;
    desc->queue_head.prev = &desc->queue_head;
    desc->wq.callback = adc_handle_state;
    desc->wq.arg = desc;

    desc->adc->ctrla &= ~ADC_CTRLA_ENABLE;
    while (desc->adc->syncbusy)
        ;

    desc->adc->ctrla |= ADC_CTRLA_SWRST;
    while (desc->adc->syncbusy)
        ;

    switch (adcnum)
    {
        case 0:
            desc->adc->calib = ADC_CALIB_BIASCOMP(NVM_SOFT_CALIB_ADC0_BIASCOMP) |
                         ADC_CALIB_BIASR2R(NVM_SOFT_CALIB_ADC0_BIASR2R) |
                         ADC_CALIB_BIASREFBUF(NVM_SOFT_CALIB_ADC0_BIASREFBUF);

            nvic_callback_set(PERIPHERAL_ID_ADC0_RESRDY, adc0_handle_interrupt);
            nvic_callback_set(PERIPHERAL_ID_ADC0, adc0_handle_interrupt);
            nvic_enable(PERIPHERAL_ID_ADC0);
            nvic_enable(PERIPHERAL_ID_ADC0_RESRDY);
            break;

        case 1:
            desc->adc->calib = ADC_CALIB_BIASCOMP(NVM_SOFT_CALIB_ADC1_BIASCOMP) |
                         ADC_CALIB_BIASR2R(NVM_SOFT_CALIB_ADC1_BIASR2R) |
                         ADC_CALIB_BIASREFBUF(NVM_SOFT_CALIB_ADC1_BIASREFBUF);

            nvic_callback_set(PERIPHERAL_ID_ADC1_RESRDY, adc1_handle_interrupt);
            nvic_callback_set(PERIPHERAL_ID_ADC1, adc1_handle_interrupt);
            nvic_enable(PERIPHERAL_ID_ADC1);
            nvic_enable(PERIPHERAL_ID_ADC1_RESRDY);
            break;
    }

    desc->adc->dbgctrl = ADC_DBGCTRL_DBGRUN;

    // 4 Sample clocks + temp/voltage compensation (+3 clocks)
    // at 16Mhz (48Mhz / 4 PRESC) rate is ~842ksps
    desc->adc->sampctrl = ADC_SAMPCTRL_OFFCOMP |
                          ADC_SAMPCTRL_SAMPLEN(3);
    desc->adc->ctrla = ADC_CTRLA_PRESCALER_DIV4 |
                       ADC_CTRLA_R2R;
    while (desc->adc->syncbusy)
        ;

    desc->adc->ctrla |= ADC_CTRLA_ENABLE;
    while (desc->adc->syncbusy)
        ;

    return 0;
}

/**
 * Given a ADC queue entry, schedule it for processing.
 *
 * @param   entry   ADC queue entry pointer.
 *
 * @returns None
 */
int adc_start(adc_desc_t *desc, adc_queue_entry_t *entry)
{
    adc_queue_add(desc, entry);

    if (desc->state == ADC_DRV_STATE_IDLE)
    {
        workqueue_add(&desc->wq, 0);
    }

    return 0;
}


#endif /* defined(__ATSAMD53__) */

