/*
 * saml_i2c.c
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
#include <stdlib.h>
#include <string.h>

#include "saml_sercom.h"

#include <vectors.h>
#include <console.h>

#include "saml_vectors.h"


// We need to use a static inline function for writes as the compiler will perform
// a read first when not optimized which will screw up the hardware.
static inline void i2c_data_write(sercom_i2c_t *i2c, uint32_t data)
{
    uint32_t addr = (uint32_t)&i2c->data;

    asm volatile ("str %0, [%1];"
                  :
                  : "r"(data), "r"(addr)
                  :
                 );
}

// We need to use a static inline function for writes as the compiler will perform
// a read first when not optimized which will screw up the hardware.
static inline uint32_t i2c_data_read(sercom_i2c_t *i2c)
{
    uint32_t addr = (uint32_t)&i2c->data;
    uint32_t data;

    asm volatile ("ldr %0, [%1];"
                  : "=r"(data)
                  : "r"(addr)
                  : "memory"
                 );

    return data;
}

void i2c_slave_cmd(sercom_i2c_t *i2c, uint32_t cmd)
{
    i2c->ctrlb = (i2c->ctrlb & ~SERCOM_I2C_CTRLB_CMD_MASK) | cmd;
}

twi_drv_t *twi_drv[SERCOM_COUNT] = {};


/*
 * Slave
 */
void twi_slave_send(twi_drv_t *twi, uint8_t data)
{
    i2c_data_write(twi->i2c, data);
}

void twi_slave_handle(twi_drv_t *twi)
{
    sercom_i2c_t *i2c = twi->i2c;

    switch (twi->state)
    {
        case TWI_DRV_STATE_IDLE:
            if (!i2c->intflag)
            {
                break;
            }

            twi->state = TWI_DRV_STATE_SLAVE_BUSY;
            if (twi->start)
            {
                twi->start(twi, twi->slave_arg);
                twi->read_len = 0;
            }

        case TWI_DRV_STATE_SLAVE_BUSY:
            if (i2c->intflag & SERCOM_I2C_INTFLAG_DRDY)
            {
                if (i2c->status & SERCOM_I2C_STATUS_DIR)
                {
                    // Read from host
                    if ((i2c->status & SERCOM_I2C_STATUS_RXNACK) &&
                         twi->read_len)
                    {
                        if (twi->nak)
                        {
                            twi->nak(twi, twi->slave_arg);
                        }

                        i2c->intflag = SERCOM_I2C_INTFLAG_PREC | SERCOM_I2C_INTFLAG_DRDY;
                        twi->state = TWI_DRV_STATE_IDLE;
                    }
                    else if (twi->read)
                    {
                        twi->read(twi, twi->slave_arg);
                        twi->read_len++;
                    }
                }
                else
                {
                    // Write from host
                    if (twi->write)
                    {
                        twi->write(twi, twi->slave_arg, i2c_data_read(i2c));
                    }
                }

                break;
            }

            if (i2c->intflag & SERCOM_I2C_INTFLAG_PREC)
            {
                if (twi->done)
                {
                    twi->done(twi, twi->slave_arg);
                }

                i2c->intflag = SERCOM_I2C_INTFLAG_PREC;
                twi->state = TWI_DRV_STATE_IDLE;
            }
            break;

        default:
            break;
    }
}

void i2c_slave_init(twi_drv_t *twi)
{
    sercom_i2c_t *i2c = twi->i2c;
    uint32_t state = irq_save();

    twi->type = TWI_DRV_TYPE_SLAVE;

    // Reset peripheral, wait for completion
    i2c->ctrla = SERCOM_I2C_CTRLA_SWRST;
    while (i2c->syncbusy)
        ;

    if (!twi->slave_addr)
    {
        irq_restore(state);
        return;
    }

    // Setup for slow mode master at provided baudrate
    i2c->ctrla = SERCOM_I2C_CTRLA_SLAVE |
                 SERCOM_I2C_CTRLA_SCLSM |
                 SERCOM_I2C_CTRLA_SPEED_SM |
                 SERCOM_I2C_CTRLA_SDAHOLD_DIS;
    i2c->ctrlb = SERCOM_I2C_CTRLB_SMEN |
                 SERCOM_I2C_CTRLB_AACKEN |
                 SERCOM_I2C_CTRLB_AMODE_MASK;
    i2c->intenset = SERCOM_I2C_INTENSET_DRDY | SERCOM_I2C_INTENSET_AMATCH |
                    SERCOM_I2C_INTENSET_PREC | SERCOM_I2C_INTENSET_ERROR;
    i2c->addr = SERCOM_I2C_ADDR_ADDR(twi->slave_addr >> 1) |
                SERCOM_I2C_ADDR_ADDRMASK(0x3ff);

    // Enable i2c
    i2c->ctrla |= SERCOM_I2C_CTRLA_ENABLE;
    while (i2c->syncbusy)
        ;

    irq_restore(state);
}

/*
 * Master
 */
void i2c_master_cmd(sercom_i2c_t *i2c, uint32_t cmd)
{
    i2c->ctrlb = (i2c->ctrlb & ~SERCOM_I2C_CTRLB_CMD_MASK) | cmd;
}

void twi_master_error(twi_drv_t *twi, int result)
{
    sercom_i2c_t *i2c = twi->i2c;

    twi->state = TWI_DRV_STATE_IDLE;
    i2c_master_cmd(i2c, SERCOM_I2C_CTRLB_CMD_STOP);

    if (twi->complete)
    {
        twi->complete(twi, twi->master_arg, -1);
        i2c_slave_init(twi);
    }

    // TODO: Go back to slave mode
}

void twi_master_handle(twi_drv_t *twi)
{
    sercom_i2c_t *i2c = twi->i2c;
    uint8_t status = i2c->status;

    // ACK interrupts
    i2c->intflag = i2c->intflag;

    // Clear errors
    i2c->status = (SERCOM_I2C_STATUS_BUSERR | SERCOM_I2C_STATUS_LENERR |
                   SERCOM_I2C_STATUS_SEXTTOUT | SERCOM_I2C_STATUS_MEXTTOUT |
                   SERCOM_I2C_STATUS_LOWTOUT | SERCOM_I2C_STATUS_ARBLOST);

    if (status & (SERCOM_I2C_STATUS_BUSERR | SERCOM_I2C_STATUS_LENERR |
                  SERCOM_I2C_STATUS_SEXTTOUT | SERCOM_I2C_STATUS_MEXTTOUT |
                  SERCOM_I2C_STATUS_LOWTOUT | SERCOM_I2C_STATUS_ARBLOST))
    {
        twi_master_error(twi, -2);
        return;
    }

    switch (twi->state)
    {
        case TWI_DRV_STATE_WRITE_DATA:
            // Check for NACK
            if (status & SERCOM_I2C_STATUS_RXNACK)
            {
                twi_master_error(twi, -1);
                break;
            }

            if (twi->write_len)  // Done writing?
            {
                // Write data
                i2c_data_write(i2c, *twi->write_buffer++);
                twi->write_len--;
            }
            else
            {
                if (!twi->read_len)
                {
                    twi->state = TWI_DRV_STATE_IDLE;
                    i2c_master_cmd(i2c, SERCOM_I2C_CTRLB_CMD_STOP);

                    if (twi->complete)
                    {
                        twi->complete(twi, twi->master_arg, 0);
                        i2c_slave_init(twi);
                    }
                    break;
                }

                // More stuff to read, restart and setup the next byte for reading
                // Start address transmit, set low bit for read
                i2c->addr = SERCOM_I2C_ADDR_MASTER_ADDR(twi->dst_addr | 0x1);
                i2c_master_cmd(i2c, SERCOM_I2C_CTRLB_CMD_ACK_RESTART);

                twi->state = TWI_DRV_STATE_READ_DATA;
            }
            break;

        case TWI_DRV_STATE_READ_DATA:
            // Check for NACK
            if (status & SERCOM_I2C_STATUS_RXNACK)
            {
                twi_master_error(twi, -1);
                break;
            }

            if (twi->read_len <= 1)
            {
                i2c->ctrlb |= SERCOM_I2C_CTRLB_ACKACT;
            }

            twi->read_len--;
            if (twi->read_len)
            {
                *twi->read_buffer++ = i2c_data_read(i2c);
                break;
            }

            // Done
            i2c_master_cmd(i2c, SERCOM_I2C_CTRLB_CMD_STOP);
            *twi->read_buffer++ = i2c_data_read(i2c);
            twi->state = TWI_DRV_STATE_IDLE;

            if (twi->complete)
            {
                twi->complete(twi, twi->master_arg, 0);
                i2c_slave_init(twi);
            }
            break;

        default:
            break;
    }
}

static void sercom_twi_int_handler(twi_drv_t *twi)
{
    if (twi->type == TWI_DRV_TYPE_MASTER)
    {
        twi_master_handle(twi);
    }
    else
    {
        twi_slave_handle(twi);
    }
}

// Interrupt handler callbacks
static void sercom_i2c0_int_handler(void)
{
    sercom_twi_int_handler(twi_drv[0]);
}

static void sercom_i2c1_int_handler(void)
{
    sercom_twi_int_handler(twi_drv[1]);
}

static void sercom_i2c2_int_handler(void)
{
    sercom_twi_int_handler(twi_drv[2]);
}

static void sercom_i2c3_int_handler(void)
{
    sercom_twi_int_handler(twi_drv[3]);
}

static void sercom_i2c4_int_handler(void)
{
    sercom_twi_int_handler(twi_drv[4]);
}

static void sercom_i2c5_int_handler(void)
{
    sercom_twi_int_handler(twi_drv[5]);
}

void i2c_master_init(twi_drv_t *twi)
{
    sercom_i2c_t *i2c = twi->i2c;
    uint32_t state = irq_save();

    twi->type = TWI_DRV_TYPE_MASTER;

    // Reset peripheral, wait for completion
    i2c->ctrla = SERCOM_I2C_CTRLA_SWRST;
    while (i2c->syncbusy)
        ;

    // Setup for slow mode master at provided baudrate
    i2c->ctrla = SERCOM_I2C_CTRLA_MASTER |
                 SERCOM_I2C_CTRLA_SPEED_SM |
                 SERCOM_I2C_CTRLA_INACTOUT_DIS |
                 SERCOM_I2C_CTRLA_SDAHOLD_DIS;
    i2c->ctrlb = SERCOM_I2C_CTRLB_SMEN;
    i2c->baud = SERCOM_I2C_BAUD_BAUD(twi->clockrate / (10 + (2 * twi->baudrate)));
    i2c->intenset = SERCOM_I2C_INTENSET_MB | SERCOM_I2C_INTENSET_SB |
                    SERCOM_I2C_INTENSET_ERROR;

    // Enable i2c
    i2c->ctrla |= SERCOM_I2C_CTRLA_ENABLE;
    while (i2c->syncbusy)
        ;

    // Force the bus state to IDLE or else the first transmission will error
    i2c->status = SERCOM_I2C_STATUS_BUSSTATE_IDLE;
    while (i2c->syncbusy)
        ;

    irq_restore(state);
}

int twi_init(twi_drv_t *twi, uint8_t peripheral_id, uint32_t clockrate, uint32_t baudrate)
{
    sercom_i2c_t *i2c;
    void (*vector)(void);

    switch (peripheral_id)
    {
        case PERIPHERAL_ID_SERCOM0:
            i2c = SERCOM0_I2C;
            vector = sercom_i2c0_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM1:
            i2c = SERCOM1_I2C;
            vector = sercom_i2c1_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM2:
            i2c = SERCOM2_I2C;
            vector = sercom_i2c2_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM3:
            i2c = SERCOM3_I2C;
            vector = sercom_i2c3_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM4:
            i2c = SERCOM4_I2C;
            vector = sercom_i2c4_int_handler;
            break;
        case PERIPHERAL_ID_SERCOM5:
            i2c = SERCOM5_I2C;
            vector = sercom_i2c5_int_handler;
            break;

        default:
            return 0;
    }

    twi->clockrate = clockrate;
    twi->baudrate = baudrate;

    twi->i2c = i2c;
    twi_drv[peripheral_id - PERIPHERAL_ID_SERCOM0] = twi;

    i2c_slave_init(twi);

    nvic_callback_set(peripheral_id, vector);
    nvic_enable(peripheral_id);

    return 0;
}

int twi_master_xfer(twi_drv_t *twi, uint8_t addr,
                    uint8_t *write_buf, uint32_t write_len,
                    uint8_t *read_buf, uint32_t read_len,
                    twi_master_callback_t complete, void *arg)
{
    sercom_i2c_t *i2c = twi->i2c;
    uint32_t irqstate = irq_save();

    if (twi->state == TWI_DRV_STATE_SLAVE_BUSY)
    {
        return -1;
        irq_restore(irqstate);
    }

    if (!write_len && !read_len)
    {
        twi->complete(twi, arg, 0);
    }

    i2c_master_init(twi);

    twi->complete = complete;
    twi->master_arg = arg;
    twi->read_buffer = read_buf;
    twi->read_len = read_len;
    twi->write_buffer = write_buf;
    twi->write_len = write_len;
    twi->dst_addr = addr;

    i2c->ctrlb &= ~SERCOM_I2C_CTRLB_ACKACT;
    if (read_len == 1)  // single byte reads needs NACK on first data
    {
        i2c->ctrlb |= SERCOM_I2C_CTRLB_ACKACT;
    }

    if (write_len)
    {
        twi->state = TWI_DRV_STATE_WRITE_DATA;

        // Start address transmit, clear low bit for write
        i2c->addr = SERCOM_I2C_ADDR_MASTER_ADDR(twi->dst_addr & 0xfe);
    }
    else
    {
        twi->state = TWI_DRV_STATE_READ_DATA;

        // Start address transmit, set low bit for read
        i2c->addr = SERCOM_I2C_ADDR_MASTER_ADDR(twi->dst_addr | 0x1);
    }

    irq_restore(irqstate);

    return 0;
}

void twi_master_wait(twi_drv_t *twi)
{
    while (twi->state != TWI_DRV_STATE_IDLE)
        ;
}

void cmd_twi_result(twi_drv_t *t, void *arg, int result)
{
    int *result_ptr = (int *)arg;

    *result_ptr = -result;
}

#define TWI_CMD_BUFLEN                           32
int cmd_twi(uart_drv_t *uart, int argc, char *argv[])
{
    uint32_t read_len = 0, write_len = 0;
    uint8_t write_buffer[TWI_CMD_BUFLEN], read_buffer[TWI_CMD_BUFLEN];
    int count, twi_index, i, result;
    uint8_t addr;
    char *result_msg[] =
    {
        "Success",
        "NACK",
        "Bus Lost",
    };

    // cmd, device number, i2c addr, read len, write bytes...
    if (argc <= 3)
    {
        if ((argc == 2) && (!strcmp(argv[1], "list")))
        {
            console_print("Available devices\r\n");
            for (i = 0; i < SERCOM_COUNT; i++)
            {
                if (twi_drv[i])
                {
                    console_print("  %d\r\n", i);
                }
            }

            return 0;
        }

        cmd_help_usage(uart, argv[0]);
        return 0;
    }

    count = 1;

    // TWI Index parameter
    twi_index = strtoul(argv[count++], NULL, 0);
    if (twi_index >= SERCOM_COUNT)
    {
        console_print("Invalid TWI device\r\n");
        return 0;
    }

    if (!twi_drv[twi_index])
    {
        console_print("TWI device not initialized\r\n");
        return 0;
    }

    // Destination Addr
    addr = strtoul(argv[count++], NULL, 0);

    // Read length parameter
    read_len = strtoul(argv[count++], NULL, 0);
    if (read_len > sizeof(read_buffer))
    {
        console_print("Read length must be <= %d\r\n", sizeof(read_buffer));
        return 0;
    }

    // Write data parameters
    while ((count < argc) && (write_len < sizeof(write_buffer)))
    {
        write_buffer[write_len++] = strtoul(argv[count++], NULL, 0);
    }

    if (count < argc)
    {
        console_print("Write length must be <= %d\r\n", sizeof(write_buffer));
        return -1;
    }

    if (twi_master_xfer(twi_drv[twi_index], addr, write_buffer, write_len, read_buffer, read_len,
                        cmd_twi_result, &result))
    {
        console_print("TWI Controller Busy");
        return -1;
    }

    twi_master_wait(twi_drv[twi_index]);

    console_print("Result: %s\r\n", result_msg[result]);
    if (!result && read_len)
    {
        for (i = 0; i < read_len; i++)
        {
            console_print("%02x ", read_buffer[i]);
        }
        console_print("\r\n");
    }

    return 0;
}

