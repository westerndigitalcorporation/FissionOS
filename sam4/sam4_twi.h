/*
 * sam4_twi.h
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


#ifndef __SAM4S_TWI_H__
#define __SAM4S_TWI_H__

#include <console.h>


#define CONSOLE_CMD_TWI                          \
    {                                            \
        .cmdstr = "twi",                         \
        .callback = cmd_twi,                     \
        .usage = "  twi <device> <addr> <read len> [write data ...]\r\n", \
        .help =                                  \
            "  Send and receive data on the TWI (I2c) bus.\r\n"  \
            "    device    : TWI bus number.\r\n" \
            "    addr      : I2c address (full 8-bit with ignored r/w bit)\r\n" \
            "    read len  : Number of bytes to read.\r\n" \
            "    write len : List of bytes to write.\r\n", \
    }


typedef struct twi
{
    uint32_t cr;
#define TWI_CR_START                             (1 << 0)
#define TWI_CR_STOP                              (1 << 1)
#define TWI_CR_MSEN                              (1 << 2)
#define TWI_CR_MSDIS                             (1 << 3)
#define TWI_CR_SVEN                              (1 << 4)
#define TWI_CR_SVDIS                             (1 << 5)
#define TWI_CR_QUICK                             (1 << 6)
#define TWI_CR_SWRST                             (1 << 7)
    uint32_t mmr;
#define TWI_MMR_IADRSZ_SHIFT                     8
#define TWI_MMR_IADRSZ_MASK                      (0x3 << 8)
#define TWI_MMR_IADRSZ_NONE                      ((0x0 & 0x3) << 8)
#define TWI_MMR_IADRSZ_1_BYTE                    ((0x1 & 0x3) << 8)
#define TWI_MMR_IADRSZ_2_BYTE                    ((0x2 & 0x3) << 8)
#define TWI_MMR_IADRSZ_3_BYTE                    ((0x3 & 0x3) << 8)
#define TWI_MMR_MREAD                            (1 << 12)
#define TWI_MMR_DADR(val)                        (((val) & 0x7f) << 16)
    uint32_t smr;
#define TWI_SMR_SADR(val)                        (((val) & 0x7f) << 16)
    uint32_t iadr;
#define TWI_IADR_IADR(val)                       (((val) & 0xffffff) << 0)
    uint32_t cwgr;
#define TWI_CWGR_CLDIV(val)                      (((val) & 0xff) << 0)
#define TWI_CWGR_CHDIV(val)                      (((val) & 0xff) << 8)
#define TWI_CWGR_CKDIV(val)                      (((val) & 0x07) << 16)
    uint32_t resvd_0x14[3];
    uint32_t sr;
#define TWI_SR_TXCOMP                            (1 << 0)
#define TWI_SR_RXRDY                             (1 << 1)
#define TWI_SR_TXRDY                             (1 << 2)
#define TWI_SR_SVREAD                            (1 << 3)
#define TWI_SR_SVACC                             (1 << 4)
#define TWI_SR_GACC                              (1 << 5)
#define TWI_SR_OVRE                              (1 << 6)
#define TWI_SR_NACK                              (1 << 8)
#define TWI_SR_ARBLST                            (1 << 9)
#define TWI_SR_SCLWS                             (1 << 10)
#define TWI_SR_EOSVACC                           (1 << 11)
#define TWI_SR_ENDRX                             (1 << 12)
#define TWI_SR_ENDTX                             (1 << 13)
#define TWI_SR_RXBUFF                            (1 << 14)
#define TWI_SR_TXBUFE                            (1 << 15)
    uint32_t ier;
#define TWI_IER_TXCOMP                           (1 << 0)
#define TWI_IER_RXRDY                            (1 << 1)
#define TWI_IER_TXRDY                            (1 << 2)
#define TWI_IER_SVACC                            (1 << 4)
#define TWI_IER_GACC                             (1 << 5)
#define TWI_IER_OVRE                             (1 << 6)
#define TWI_IER_NACK                             (1 << 8)
#define TWI_IER_ARBLST                           (1 << 9)
#define TWI_IER_SCL_WS                           (1 << 10)
#define TWI_IER_EOSVACC                          (1 << 11)
#define TWI_IER_ENDRX                            (1 << 12)
#define TWI_IER_ENDTX                            (1 << 13)
#define TWI_IER_RXBUFF                           (1 << 14)
#define TWI_IER_TXBUFE                           (1 << 15)
    uint32_t idr;
#define TWI_IDR_TXCOMP                           (1 << 0)
#define TWI_IDR_RXRDY                            (1 << 1)
#define TWI_IDR_TXRDY                            (1 << 2)
#define TWI_IDR_SVACC                            (1 << 4)
#define TWI_IDR_GACC                             (1 << 5)
#define TWI_IDR_OVRE                             (1 << 6)
#define TWI_IDR_NACK                             (1 << 8)
#define TWI_IDR_ARBLST                           (1 << 9)
#define TWI_IDR_SCL_WS                           (1 << 10)
#define TWI_IDR_EOSVACC                          (1 << 11)
#define TWI_IDR_ENDRX                            (1 << 12)
#define TWI_IDR_ENDTX                            (1 << 13)
#define TWI_IDR_RXBUFE                           (1 << 14)
#define TWI_IDR_TXBUFE                           (1 << 15)
    uint32_t imr;
#define TWI_IMR_TXCOMP                           (1 << 0)
#define TWI_IMR_RXRDY                            (1 << 1)
#define TWI_IMR_TXRDY                            (1 << 2)
#define TWI_IMR_SVACC                            (1 << 4)
#define TWI_IMR_GACC                             (1 << 5)
#define TWI_IMR_OVRE                             (1 << 6)
#define TWI_IMR_NACK                             (1 << 8)
#define TWI_IMR_ARBLST                           (1 << 9)
#define TWI_IMR_SCL_WS                           (1 << 10)
#define TWI_IMR_EOSVACC                          (1 << 11)
#define TWI_IMR_ENDRX                            (1 << 12)
#define TWI_IER_ENDTX                            (1 << 13)
#define TWI_IMR_RXBUFF                           (1 << 14)
#define TWI_IMR_TXBUFE                           (1 << 15)
    uint32_t rhr;
    uint32_t thr;
} __attribute__ ((packed)) twi_t;

#if defined(__AT91SAM4S__)
#define TWI0                                     ((volatile twi_t *)0x40018000)
#define TWI1                                     ((volatile twi_t *)0x4001c000)
#elif defined(__AT91SAM4E__)
#define TWI0                                     ((volatile twi_t *)0x400a8000)
#define TWI1                                     ((volatile twi_t *)0x400ac000)
#endif
#define TWI_HW_MODULES                           2


#define TWI_RESULT_OK                            0
#define TWI_RESULT_NAK                           1
#define TWI_RESULT_OVERRUN                       2
#define TWI_RESULT_ARBLOST                       3

struct twi_drv;

/*
 * All callbacks occur in interrupt context, applications should defer processing
 * to workqueues or tasks.
 */
typedef void (*twi_master_callback_t)(struct twi_drv *twi, void *arg, int result);
typedef void (*twi_slave_rx_t)(struct twi_drv *twi, void *arg, uint8_t data);
typedef void (*twi_slave_tx_t)(struct twi_drv *twi, void *arg);
typedef void (*twi_slave_start_t)(struct twi_drv *twi, void *arg);
typedef void (*twi_slave_done_t)(struct twi_drv *twi, void *arg);
typedef void (*twi_slave_nak_t)(struct twi_drv *twi, void *arg);

typedef struct twi_drv
{
    /* The following need to be iniialized by the user */
    volatile twi_t *twi;

    volatile uint8_t type;
#define TWI_DRV_TYPE_MASTER                       0
#define TWI_DRV_TYPE_SLAVE                        1

    volatile uint32_t state;
#define TWI_DRV_STATE_IDLE                        0
#define TWI_DRV_STATE_WRITE                       1
#define TWI_DRV_STATE_STOP                        2
#define TWI_DRV_STATE_READ                        3
#define TWI_DRV_STATE_SLAVE_BUSY                  4

    uint8_t slave_addr;
    uint8_t dst_addr;

    /* Master */
    twi_master_callback_t complete;
    void *master_arg;
    uint32_t read_len;
    uint8_t *read_buffer;
    uint32_t write_len;
    uint8_t *write_buffer;

    /* Slave */
    twi_slave_start_t start;
    twi_slave_tx_t read;
    twi_slave_rx_t write;
    twi_slave_done_t done;
    twi_slave_nak_t nak;
    void *slave_arg;
} twi_drv_t;


int twi_init(twi_drv_t *twi, uint8_t peripheral_id);
int twi_master_xfer(twi_drv_t *twi, uint8_t addr,
                    uint8_t *write_buf, uint32_t write_len,
                    uint8_t *read_buf, uint32_t read_len,
                    twi_master_callback_t complete, void *arg);
void twi_master_wait(twi_drv_t *twi);

void twi_slave_send(twi_drv_t *twi, uint8_t data);

int cmd_twi(console_t *console, int argc, char *argv[]);

#endif /* __SAM4S_I2C_H__ */

