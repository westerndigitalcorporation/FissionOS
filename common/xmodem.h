/*
 * swd_driver.h
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


#ifndef __XMODEM_H__
#define __XMODEM_H__

/*
 *
 * XModem Packet Format - Sender
 *
 * Byte 0  Byte 1     Byte 2       Byte 3-130  Byte 131
 * ------  ---------  -----------  ----------  -------------
 * SOH     Packet #   ~(Packet #)  Data        8-bit Checksum
 *
 * Xmodem CRC Packet Format - Receiver
 *
 * Byte 0
 * ------
 * ACK/NAK
 *
 * Transfer starts by the receiver sending NAKs at 1 second intervals.
 * After the receiving the NAK, the sender transmits a full data packet.
 * On frame reception, the receiver will either ACK or NAK.  A ACK
 * character is transmitted when a full packet with valid checksum is
 * received.  A NAK is transmitted due to the following:
 *
 * - Checksum Error
 * - Incomplete packet  (within 3 seconds after sucessful first packet)
 * - Missing packet     (within 3 seconds after sucessful first packet)
 *
 * Each packet must be 132 bytes.  The final packet from the sender must
 * be padded if less than a full frame.  The transmission is terminated
 * when the sender sends a EOT character.
 *
 */


#define XMODEM_DATA_MAX_LEN                      128


struct xmodem;
typedef void (*xmodem_recv_cb_t)(struct xmodem *, void *arg, uint8_t *data, uint8_t len);

typedef struct xmodem_pkt
{
    uint8_t soh;
    uint8_t num;
    uint8_t inv_num;
    uint8_t data[XMODEM_DATA_MAX_LEN];
    uint8_t cksum;
} __attribute__ ((packed)) xmodem_pkt_t;

typedef struct xmodem
{
    uint32_t timeout;
    void *arg;
    xmodem_recv_cb_t recv_cb;
    uint8_t state;
#define XMODEM_STATE_IDLE                        0
#define XMODEM_STATE_START                       1
#define XMODEM_STATE_SOH                         2
    uint8_t packet_num;
    uint8_t len;
    uint8_t nak_count;
    xmodem_pkt_t packet;
} xmodem_t;


//
// Must be implemented when porting
//
extern uint64_t xmodem_system_time_ms(void);
extern uint32_t xmodem_send(void *arg, uint8_t *data, uint8_t len);


//
// API
//
void xmodem_init(xmodem_t *xmodem, void *arg, xmodem_recv_cb_t recv_cb);
void xmodem_start(xmodem_t *xmodem);
int xmodem_timer(xmodem_t *xmodem);
int xmodem_recv(xmodem_t *xmodem, uint8_t *data, uint8_t len);


#endif /* __XMODEM_H__ */
