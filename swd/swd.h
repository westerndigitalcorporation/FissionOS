/*
 * swd.h
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


#ifndef __SWD_H__
#define __SWD_H__


#define SWD_INIT_CLK_CYCLES                      56
#define SWD_INIT_SEQUENCE                        { 0x9e, 0xe7 }


#define SWD_WRITE                                0
#define SWD_READ                                 1

#define SWD_DP                                   0
#define SWD_AP                                   1

/* Request phase control bits */
#define SWD_REQUEST_START                        (1 << 0)
#define SWD_REQUEST_AP                           (1 << 1)
#define SWD_REQUEST_READ                         (1 << 2)
#define SWD_REQUEST_ADDR(val)                    ((val & 0x3) << 3)
#define SWD_REQUEST_PARITY                       (1 << 5)
#define SWD_REQUEST_STOP                         (1 << 6)
#define SWD_REQUEST_PARK                         (1 << 7)

/* ACK phase return codes */
#define SWD_REQUEST_ACK_ACK                      0x1
#define SWD_REQUEST_ACK_WAIT                     0x2
#define SWD_REQUEST_ACK_FAULT                    0x4
#define SWD_REQUEST_ACK_DISCONNECT               0x7

/* DP Registers */
#define SWD_DP_ABORT                             0x0    // Write
#define   SWD_DP_ABORT_ALL                       0x1f
#define SWD_DP_IDCODE                            0x0    // Read
#define SWD_DP_CTRLSTAT                          0x4
#define   SWD_DP_CTRLSTAT_SYS_POWERUP_ACK        (1UL << 31)
#define   SWD_DP_CTRLSTAT_SYS_POWERUP_REQ        (1UL << 30)
#define   SWD_DP_CTRLSTAT_DBG_POWERUP_ACK        (1UL << 29)
#define   SWD_DP_CTRLSTAT_DBG_POWERUP_REQ        (1UL << 28)
#define   SWD_DP_CTRLSTAT_READOK                 (1UL << 6)
#define SWD_DP_APSELECT                          0x8
#define   SWD_DP_APSELECT_APBANK(val)            (val & 0xf0)
#define   SWD_DP_APSELECT_APSEL(val)             (val << 24)
#define SWD_DP_READBUFFER                        0xc

/* AP Registers */
#define SWD_AP_CSW                               0x00
#define   SWD_AP_CSW_DBGSTATUS                   (1UL << 6)
#define   SWD_AP_CSW_HPROT1                      (1UL << 25)
#define   SWD_AP_CSW_DBGMASTER                   (1UL << 29)
#define   SWD_AP_CSW_32BIT                       ((0x2 & 0x3) << 0)
#define   SWD_AP_CSW_16BIT                       ((0x1 & 0x3) << 0)
#define   SWD_AP_CSW_8BIT                        ((0x0 & 0x3) << 0)
#define SWD_AP_TAR                               0x04
#define SWD_AP_DRW                               0x0c
#define SWD_AP_IDR                               0xfc


/* Error codes */
#define SWD_ERROR_ACK                            1
#define SWD_ERROR_PARITY                         2
#define SWD_ERROR_HARDWARE                       3

struct swd;
typedef int (*swd_driver_start)(struct swd *instance);
typedef int (*swd_driver_stop)(struct swd *instance);
typedef void (*swd_driver_flush)(struct swd *instance);
typedef int (*swd_driver_request_xmit)(struct swd *instance, uint8_t request, uint8_t *ack, uint32_t data);
typedef int (*swd_driver_request_recv)(struct swd *instance, uint8_t request, uint8_t *ack, uint32_t *data);

typedef struct swd
{
    void *driver_private;

    swd_driver_start start;
    swd_driver_stop stop;
    swd_driver_flush flush;
    swd_driver_request_xmit request_xmit;
    swd_driver_request_recv request_recv;
} swd_t;

int swd_start(swd_t *instance);
int swd_stop(swd_t *instance);

int swd_mem_write(swd_t *instance, uint32_t addr, uint32_t data);
int swd_mem_read(swd_t *instance, uint32_t addr, uint32_t *data);

#endif /* __SWD_H__ */
