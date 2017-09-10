/*
 * sam4_twi.c
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


#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#include "vectors.h"

#include "sam4_flash.h"
#include "sam4_clock.h"
#include "sam4_gpio.h"
#include "sam4_twi.h"

#include "console.h"

/*
 * Clock formula
 *
 * MCK / (((CLDIV * (2 ^ CKDIV)) + 4)
 *
 * Using the above formula and a MCK of 96Mhz, we can use CHDIV/CLDIV = 15, CKDIV = 5 to get 100kHz
 */
#define TWI_CHDIV                                15
#define TWI_CLDIV                                15
#define TWI_CKDIV                                5

#define TWI_DRV_0                                0
#define TWI_DRV_1                                1

#define TWI0_TCK_PORT                            GPIOA
#define TWI0_TCK_PIN                             4
#define TWI0_TWD_PORT                            GPIOA
#define TWI0_TWD_PIN                             3

#define TWI1_TCK_PORT                            GPIOB
#define TWI1_TCK_PIN                             5
#define TWI1_TWD_PORT                            GPIOB
#define TWI1_TWD_PIN                             4

static twi_drv_t *twis[TWI_HW_MODULES] = { NULL };

// Only set when starting a master transaction.  Driver defaults
// to slave mode if a slave address was specified.
//
// Must be called with interrupts disabled or inside interrupt
// handler to avoid function preemption.
static void twi_set_master(twi_drv_t *twi)
{
    twi->type = TWI_DRV_TYPE_MASTER;
    twi->state = TWI_DRV_STATE_WRITE;

    twi->twi->idr = TWI_IDR_TXBUFE | TWI_IDR_RXBUFE | TWI_IDR_ENDTX | TWI_IDR_ENDRX |
                    TWI_IDR_RXRDY | TWI_IDR_SVACC | TWI_IDR_EOSVACC | TWI_IDR_TXCOMP;
    twi->twi->ier = TWI_SR_NACK | TWI_SR_OVRE | TWI_SR_ARBLST;

    if (twi->write_len == 0)
    {
        twi->state = TWI_DRV_STATE_STOP;
    }
    else if ((twi->write_len <= 3) && (twi->read_len))
    {
        /*
         * HACK HACK HACK - Due to SAM4 I2c hardware limitations
         *
         * The only way to get a SAM4 to issue restarts is using the internal address
         * feature.  There is no generic restart mechanism, and it only works if you
         * have fewer or equal to 3 send bytes prior to read.  This greatly uglifies
         * the state machine, but here it is.
         */
        int i;

        twi->twi->cr = TWI_CR_SVDIS | TWI_CR_MSEN;

        twi->state = TWI_DRV_STATE_READ;
        twi->twi->ier = TWI_IER_RXRDY;
        twi->twi->mmr = TWI_MMR_DADR(twi->dst_addr >> 1) |
                        (twi->write_len << TWI_MMR_IADRSZ_SHIFT) |
                        TWI_MMR_MREAD;

        for (i = 0; i < twi->write_len; i++)
        {
            twi->twi->iadr = (twi->twi->iadr << 8) | *twi->write_buffer++;
        }

        twi->twi->cr = TWI_CR_START | ((twi->read_len == 1) ? TWI_CR_STOP : 0);

        return;
    }

    twi->twi->cr = TWI_CR_SVDIS | TWI_CR_MSEN;
}


// Must be called with interrupts disabled or inside interrupt
// handler to avoid preemption.
static void twi_set_slave(twi_drv_t *twi)
{
    twi->type = TWI_DRV_TYPE_SLAVE;
    twi->state = TWI_DRV_STATE_IDLE;

    twi->twi->cr = TWI_CR_SWRST;

    // Setup the master clock frequency @ 100Khz
    twi->twi->cwgr = TWI_CWGR_CLDIV(TWI_CLDIV) | TWI_CWGR_CHDIV(TWI_CHDIV) |
                     TWI_CWGR_CKDIV(TWI_CKDIV);

    twi->twi->idr = TWI_IDR_TXBUFE | TWI_IDR_RXBUFE | TWI_IDR_ENDTX | TWI_IDR_ENDRX |
                    TWI_IDR_EOSVACC | TWI_IER_TXCOMP | TWI_IDR_RXRDY |
                    TWI_IDR_TXRDY | TWI_IDR_SVACC | TWI_IDR_NACK |
                    TWI_IDR_OVRE | TWI_IDR_ARBLST;

    // Slave is only set if we have a i2c slave address
    if (twi->slave_addr)
    {
        twi->twi->smr = TWI_SMR_SADR(twi->slave_addr >> 1);

        twi->twi->ier = TWI_IER_SVACC | TWI_IER_TXRDY;
        twi->twi->cr = TWI_CR_MSDIS | TWI_CR_SVEN;

        (void)twi->twi->rhr;  // Read RHR to clear it out
    }
}

static void twi_master_interrupt(twi_drv_t *twi)
{
    uint32_t status = twi->twi->sr;
    int result = 0;

    // Check for transmission/reception error, applies to all states
    if (status & (TWI_SR_NACK | TWI_SR_OVRE | TWI_SR_ARBLST))
    {
        if (status & TWI_SR_NACK)
        {
            result = -TWI_RESULT_NAK;
        }
        else if (status & TWI_SR_OVRE)
        {
            result = -TWI_RESULT_OVERRUN;
        }
        else if (status & TWI_SR_ARBLST)
        {
            result = -TWI_RESULT_ARBLOST;
        }

        goto done;
    }

    // Do state machine processing
    switch (twi->state)
    {
        case TWI_DRV_STATE_WRITE:
            if (!(status & TWI_SR_TXRDY))
            {
                // Hardware workaround.  For some reason, we get a interrupt with
                // sr set to 0xf008, but the imr is 0x0344.  No overlap and no
                // apparent reason to wake up, so ignore.  This only happens before
                // transmission of the first byte is complete.
                return;
            }

            if (twi->write_len)
            {
                twi->twi->ier = TWI_IER_TXRDY;
                twi->twi->mmr = TWI_MMR_IADRSZ_NONE | TWI_MMR_DADR(twi->dst_addr >> 1);

                twi->twi->thr = *twi->write_buffer++;
                twi->write_len--;
            }
            else  // Send stop if last byte
            {
                twi->state = TWI_DRV_STATE_STOP;

                twi->twi->idr = TWI_IDR_TXRDY;
                twi->twi->ier = TWI_IDR_TXCOMP;

                twi->twi->cr = TWI_CR_STOP;
            }

            return;

        case TWI_DRV_STATE_STOP:
            if (!(status & TWI_SR_TXCOMP))
            {
                // Check for spurious interrupt
                return;
            }

            // Transition to read
            if (twi->read_len)
            {
                twi->state = TWI_DRV_STATE_READ;

                twi->twi->idr = TWI_IDR_TXCOMP;   // Turn off transmit complete
                twi->twi->ier = TWI_IER_RXRDY;    // Turn on receive full
                twi->twi->mmr = TWI_MMR_IADRSZ_NONE | TWI_MMR_DADR(twi->dst_addr >> 1) |
                                TWI_MMR_MREAD;
                twi->twi->cr = TWI_CR_START | ((twi->read_len == 1) ? TWI_CR_STOP : 0);

                return;
            }

            // Complete
            break;

        case TWI_DRV_STATE_READ:
            if (!(status & TWI_SR_RXRDY))
            {
                // Check for spurious interrupt
                return;
            }

            *twi->read_buffer = twi->twi->rhr & 0xff;
            twi->read_buffer++;

            twi->read_len--;
            if (twi->read_len == 1)
            {
                twi->twi->cr = TWI_CR_STOP;
            }

            if (twi->read_len)
            {
                return;
            }

            // Complete
            break;
    }

done:
    twi_set_slave(twi);

    if (twi->complete)
    {
        twi->complete(twi, twi->master_arg, result);
    }
}

int twi_master_xfer(twi_drv_t *twi, uint8_t addr,
                    uint8_t *write_buf, uint32_t write_len,
                    uint8_t *read_buf, uint32_t read_len,
                    twi_master_callback_t complete, void *arg)
{
    volatile uint32_t irq;

    irq = irq_save();

    if (twi->state != TWI_DRV_STATE_IDLE)
    {
        irq_restore(irq);
        return -1;
    }

    twi->complete = complete;
    twi->master_arg = arg;
    twi->write_buffer = write_buf;
    twi->write_len = write_len;
    twi->read_buffer = read_buf;
    twi->read_len = read_len;
    twi->dst_addr = addr;

    twi_set_master(twi);                             // Set Master Mode

    // Initiate the transfer
    twi_master_interrupt(twi); // Process the first byte

    irq_restore(irq);

    return 0;
}

void twi_master_wait(twi_drv_t *twi)
{
    while (twi->state != TWI_DRV_STATE_IDLE)
        ;
}

void twi_slave_send(twi_drv_t *twi, uint8_t data)
{
    twi->twi->thr = data;
}

static void twi_slave_interrupt(twi_drv_t *twi)
{
    uint32_t status = twi->twi->sr;

    // Start of i2c transaction
    if ((status & TWI_SR_SVACC) && (twi->state == TWI_DRV_STATE_IDLE))
    {
        twi->state = TWI_DRV_STATE_SLAVE_BUSY;
        twi->twi->idr = TWI_IDR_SVACC;

        if (twi->start)
        {
            twi->start(twi, twi->slave_arg);
        }

        // Setup interrupt mask for expected transaction
        twi->twi->ier = TWI_IER_TXCOMP | TWI_IER_OVRE | TWI_IER_RXRDY | TWI_IER_NACK;
    }

    if (status & TWI_SR_RXRDY)
    {
        uint8_t data = twi->twi->rhr;

        if (twi->write)
        {
            twi->write(twi, twi->slave_arg, data);
        }
    }

    if (status & TWI_SR_TXRDY)
    {
        if (twi->read)
        {
            twi->read(twi, twi->slave_arg);
        }
        else
        {
            // Send something or we'll clock stretch forever
            twi_slave_send(twi, 0);
        }
    }

    if (status & TWI_SR_NACK)
    {
        if (twi->nak)
        {
            twi->nak(twi, twi->slave_arg);
        }

        (void)twi->twi->rhr;  // ACK the NACK int
    }

    if (status & TWI_SR_TXCOMP)
    {
        twi->twi->idr = TWI_IDR_TXCOMP | TWI_IDR_OVRE | TWI_IDR_RXRDY | TWI_IDR_NACK;

        if (twi->done)
        {
            twi->done(twi, twi->slave_arg);
        }

        twi->state = TWI_DRV_STATE_IDLE;
        twi->twi->ier = TWI_IER_SVACC;
    }
}

static void twi_interrupt(int twi_devnum)
{
    if (twis[twi_devnum])
    {
        switch (twis[twi_devnum]->type)
        {
            case TWI_DRV_TYPE_MASTER:
                twi_master_interrupt(twis[twi_devnum]);
                break;

            case TWI_DRV_TYPE_SLAVE:
                twi_slave_interrupt(twis[twi_devnum]);
                break;

            default:
                return;
        }

    }
}

static void twi0_interrupt(void)
{
    twi_interrupt(TWI_DRV_0);
}

static void twi1_interrupt(void)
{
    twi_interrupt(TWI_DRV_1);
}

int twi_init(twi_drv_t *twi, uint8_t peripheral_id)
{
    switch (peripheral_id)
    {
        case PERIPHERAL_ID_TWI0:
            twis[TWI_DRV_0] = twi;

            nvic_callback_set(PERIPHERAL_ID_TWI0, twi0_interrupt);

            GPIO_PERIPHERAL_SET(TWI0_TCK_PORT, TWI0_TCK_PIN, GPIO_PERIPHERAL_A);
            GPIO_PULLUP_DISABLE(TWI0_TCK_PORT, TWI0_TCK_PIN);
            GPIO_DISABLE(TWI0_TCK_PORT, TWI0_TCK_PIN);

            GPIO_PERIPHERAL_SET(TWI0_TWD_PORT, TWI0_TWD_PIN, GPIO_PERIPHERAL_A);
            GPIO_PULLUP_DISABLE(TWI0_TWD_PORT, TWI0_TWD_PIN);
            GPIO_DISABLE(TWI0_TWD_PORT, TWI0_TWD_PIN);
            break;

        case PERIPHERAL_ID_TWI1:
            twis[TWI_DRV_1] = twi;

            jtag_disable();  // JTAG TCK and TMS overlap TWI1 pins
            nvic_callback_set(PERIPHERAL_ID_TWI1, twi1_interrupt);

            GPIO_PERIPHERAL_SET(TWI1_TCK_PORT, TWI1_TCK_PIN, GPIO_PERIPHERAL_A);
            GPIO_PULLUP_DISABLE(TWI1_TCK_PORT, TWI1_TCK_PIN);
            GPIO_DISABLE(TWI1_TCK_PORT, TWI1_TCK_PIN);

            GPIO_PERIPHERAL_SET(TWI1_TWD_PORT, TWI1_TWD_PIN, GPIO_PERIPHERAL_A);
            GPIO_PULLUP_DISABLE(TWI1_TWD_PORT, TWI1_TWD_PIN);
            GPIO_DISABLE(TWI1_TWD_PORT, TWI1_TWD_PIN);
            break;

        default:
            return -1;
    }

    clock_peripheral_start(peripheral_id);

    twi->twi->cr = TWI_CR_SWRST;

    // Setup the master clock frequency @ 100Khz
    twi->twi->cwgr = TWI_CWGR_CLDIV(TWI_CLDIV) | TWI_CWGR_CHDIV(TWI_CHDIV) |
                     TWI_CWGR_CKDIV(TWI_CKDIV);

    nvic_enable(peripheral_id);

    twi_set_slave(twi);

    return 0;
}

void cmd_twi_result(twi_drv_t *t, void *arg, int result)
{
    int *result_ptr = (int *)arg;

    *result_ptr = -result;
}

#define TWI_CMD_BUFLEN                           32
int cmd_twi(console_t *console, int argc, char *argv[])
{
    uint32_t read_len = 0, write_len = 0;
    uint8_t write_buffer[TWI_CMD_BUFLEN], read_buffer[TWI_CMD_BUFLEN];
    int count, twi_index, i, result;
    uint8_t addr;
    char *result_msg[] =
    {
        "Success",
        "NAK",
        "Overrun",
        "Bus Lost",
    };

    // cmd, device number, i2c addr, read len, write bytes...
    if (argc <= 3)
    {
        cmd_help_usage(console, argv[0]);
        return 0;
    }

    count = 1;

    // TWI Index parameter
    twi_index = strtoul(argv[count++], NULL, 0);
    if (twi_index >= TWI_HW_MODULES)
    {
        console_print(console, "Invalid TWI device\r\n");
        return 0;
    }

    if (!twis[twi_index])
    {
        console_print(console, "TWI device not initialized\r\n");
        return 0;
    }

    // Destination Addr
    addr = strtoul(argv[count++], NULL, 0);

    // Read length parameter
    read_len = strtoul(argv[count++], NULL, 0);
    if (read_len > sizeof(read_buffer))
    {
        console_print(console, "Read length must be <= %d\r\n", sizeof(read_buffer));
        return 0;
    }

    // Write data parameters
    while ((count < argc) && (write_len < sizeof(write_buffer)))
    {
        write_buffer[write_len++] = strtoul(argv[count++], NULL, 0);
    }

    if (count < argc)
    {
        console_print(console, "Write length must be <= %d\r\n", sizeof(write_buffer));
        return -1;
    }

    if (twi_master_xfer(twis[twi_index], addr, write_buffer, write_len, read_buffer, read_len,
                        cmd_twi_result, &result))
    {
        console_print(console, "TWI Controller Busy");
        return -1;
    }

    twi_master_wait(twis[twi_index]);

    console_print(console, "Result: %s\r\n", result_msg[result]);
    if (!result && read_len)
    {
        for (i = 0; i < read_len; i++)
        {
            console_print(console, "%02x ", read_buffer[i]);
        }
        console_print(console, "\r\n");
    }

    return 0;
}

