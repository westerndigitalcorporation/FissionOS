/*
 * sam4_acc.c
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


#if defined(__AT91SAM4S__)

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

#include "vectors.h"

#include "sam4_clock.h"
#include "sam4_gpio.h"
#include "sam4_adc.h"

#include "workqueue.h"


/* ADC Driver Internal Processing States */
#define ADC_DRV_STATE_IDLE                  0
#define ADC_DRV_STATE_START                 1
#define ADC_DRV_STATE_CALIBRATE             2
#define ADC_DRV_STATE_SAMPLING              3


static volatile uint32_t adc_drv_state = ADC_DRV_STATE_IDLE;
static uint32_t adc_drv_active, adc_drv_count;
static uint16_t adc_conversion_results[ADC_AVERAGE_SAMPLES];
static volatile adc_queue_entry_t adc_queue_head =
{
    .next = (adc_queue_entry_t *)&adc_queue_head,
    .prev = (adc_queue_entry_t *)&adc_queue_head,
};


/**
 * Append a new ADC sample item to the end of the sample queue.
 *
 * @param    entry  ADC sample descriptor pointer.
 *
 * @returns  None
 */
void adc_queue_add(adc_queue_entry_t *entry)
{
    volatile uint32_t irq = irq_save();

    // If already on the queue, ignore the add request
    if (entry->next || entry->prev)
    {
        irq_restore(irq);
        return;
    }

    /* link */
    entry->next = (adc_queue_entry_t *)&adc_queue_head;
    entry->prev = adc_queue_head.prev;

    adc_queue_head.prev->next = entry;
    adc_queue_head.prev = entry;

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
static adc_queue_entry_t *adc_queue_first(void)
{
    volatile uint32_t irq = irq_save();
    adc_queue_entry_t *next = adc_queue_head.next;

    if (next == &adc_queue_head)
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
static void adc_do_calc(void)
{
    adc_queue_entry_t *next = adc_queue_first();
    adc_drv_t *adc_drv = next->adcs[adc_drv_active];
    int i;

    if (adc_drv->calc)
    {
        uint32_t min = adc_conversion_results[0];
        uint32_t max = adc_conversion_results[0];
        uint64_t total = 0;

        for (i = 0; i < ADC_AVERAGE_SAMPLES; i++)
        {
            total += adc_conversion_results[i];

            if (adc_conversion_results[i] < min)
            {
                min = adc_conversion_results[i];
            }

            if (adc_conversion_results[i] > max)
            {
                max = adc_conversion_results[i];
            }
        }

        adc_drv->calc(adc_drv, adc_drv->value, total, min, max,
                      ADC_AVERAGE_SAMPLES, adc_drv->calc_arg);
    }
}

/**
 * Setup hardware to start self calibration process.
 *
 * @returns  None
 */
static void adc_calibration_start(void)
{
    adc_drv_state = ADC_DRV_STATE_CALIBRATE;

    nvic_enable(PERIPHERAL_ID_ADC);
    ADC->ier = ADC_IER_EOCAL;
    ADC->cr = ADC_CR_AUTOCAL;
}

/**
 * Setup the hardware to perform a given ADC conversion.
 *
 * @param    adc_drv  ADC driver descriptor pointer.
 *
 * @returns  None
 */
static void adc_hw_setup(adc_drv_t *adc_drv)
{
    adc_drv->total = 0;

    // The following will set the ADC to 1Msps with the majority
    // of delay time spent in settling/track/transfer phases.
    ADC->mr =   ADC_MR_PRESCAL(5)     // mck / ((val + 1) * 2)
              | ADC_MR_STARTUP(0)     // (val + 1) * 8
              | ADC_MR_SETTLING(0)    // 0 = 3, 1 = 5, 2 = 9, 3 = 17
              | ADC_MR_TRACKTIM(1)    // (val + 1)
              | ADC_MR_TRANSFER(1)    // (val * 2) + 3
              | ADC_MR_ANACH;         // different settings per channel

    ADC->chdr = ADC_CHDR_DISABLE_ALL;
    ADC->cher = (1 << adc_drv->channel);
    ADC->cgr = ((adc_drv->gain & 0x3) << (adc_drv->channel * 2));
    if (adc_drv->flags & ADC_DRV_FLAGS_DIFF)
    {
        ADC->cor = ADC_COR_DIFF_MASK(1 << adc_drv->channel);
    }
}

/**
 * Start the hardware ADC conversion.
 *
 * @returns  None
 */
static void adc_conversion_start(void)
{
    adc_drv_state = ADC_DRV_STATE_SAMPLING;

    /* Setup the DMA */
    ADC->pdc.rpr = adc_conversion_results;
    ADC->pdc.rcr = ADC_AVERAGE_SAMPLES;
    ADC->pdc.rncr = 0;
    ADC->pdc.ptcr = PDC_PTCR_RXTEN;

    /* Set free running mode so samples automatically repeat
     * for the DMA to suck up */
    ADC->mr |= ADC_MR_FREERUN;

    nvic_enable(PERIPHERAL_ID_ADC);
    ADC->ier = ADC_IER_ENDRX;
    ADC->cr = ADC_CR_START;
}

/**
 * Find and start the next ADC conversion for each channel in the set, and
 * if the set is complete, call the completion callbacks, and start the next
 * element in the queue.
 *
 * @returns  None
 */
static void adc_next(void)
{
    adc_queue_entry_t *entry = adc_queue_first();
    adc_drv_t *adc_drv = entry->adcs[adc_drv_active];

    ADC->cr = ADC_CR_SWRST;

    // If we've done the last one, call the completion routine and go
    // back to idle.
    if (!adc_drv_count)
    {
        // Dequeue
        adc_queue_remove(entry);

        // Execute completions
        if (entry->complete)
        {
            entry->complete(entry->adcs, adc_drv_active, entry->complete_arg);
        }

        // Get next
        entry = adc_queue_first();
        if (!entry)
        {
            // None, stop processing.
            adc_drv_state = ADC_DRV_STATE_IDLE;
            return;
        }

        // Start again
        adc_drv_active = 0;
        adc_drv_count = entry->count;
        adc_drv = entry->adcs[adc_drv_active];
    }

    adc_hw_setup(adc_drv);
    adc_do_setup(adc_drv);

    adc_drv_count--;

    if (adc_drv->flags | ADC_DRV_FLAGS_CALIBRATE)
    {
        adc_calibration_start();
        return;
    }

    adc_conversion_start();
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
    adc_queue_entry_t *next;

    switch (adc_drv_state)
    {
        case ADC_DRV_STATE_IDLE:
            next = adc_queue_first();
            if (!next)
            {
                break;
            }

            adc_drv_active = 0;
            adc_drv_count = next->count;
            adc_drv_state = ADC_DRV_STATE_START;

        case ADC_DRV_STATE_START:
            adc_next();
            break;

        case ADC_DRV_STATE_CALIBRATE:
            adc_conversion_start();
            break;

        case ADC_DRV_STATE_SAMPLING:
            adc_do_calc();

            adc_drv_active++;
            adc_next();
            break;
    }
}

/* ADC processing workqueue descriptor */
static workqueue_t adc_wq =
{
    .callback = adc_handle_state,
};

/**
 * Handle the ADC interrupt.  ACK and disable interrupt and schedule
 * ADC processing workqueue for processing.
 *
 * @returns  None
 */
static void adc_handle_interrupt(void)
{
    ADC->idr = ADC->isr;
    nvic_disable(PERIPHERAL_ID_ADC);

    workqueue_add(&adc_wq, 0);
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
 * Initialize the ADC driver.  Setup interrupt handler and turn on
 * peripheral clocks.
 *
 * @returns  None
 */
void adc_init(void)
{
    clock_peripheral_start(PERIPHERAL_ID_ADC);
    nvic_callback_set(PERIPHERAL_ID_ADC, adc_handle_interrupt);
}

/**
 * Given a ADC queue entry, schedule it for processing.
 *
 * @param   entry   ADC queue entry pointer.
 *
 * @returns None
 */
int adc_start(adc_queue_entry_t *entry)
{
    adc_queue_add(entry);

    if (adc_drv_state == ADC_DRV_STATE_IDLE)
    {
        workqueue_add(&adc_wq, 0);
    }

    return 0;
}


#endif /* defined(__AT91SAM4S__) */

