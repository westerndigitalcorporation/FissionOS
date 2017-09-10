/*
 * usb.h
 *
 *
 * Copyright (c) 2017 Jeremy Garff
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
 * Author: Jeremy Garff <jer@jers.net>
 *
 */


#ifndef __USB_H__
#define __USB_H__


#include "usb_messages.h"


//
// Endpoint configuration definitions
//
#define USB_ENDPOINT_CONFIG_OK     0
#define USB_ENDPOINT_CONFIG_FAIL  -1

//
// Hardware and driver specific information
//
typedef struct usb_endpoint_entry usb_endpoint_entry_t;
struct usb_endpoint_entry
{
    uint8_t  num;                         ///< The endpoing number for this configuration.
    uint8_t  type;                        ///< USB_ENDPOINT_TYPE_*
#define USB_ENDPOINT_TYPE_CONTROL  0
#define USB_ENDPOINT_TYPE_ISO      1
#define USB_ENDPOINT_TYPE_BULK     2
#define USB_ENDPOINT_TYPE_INT      3

    uint8_t  direction;                   ///< USB_ENDPOINT_DIR_*
#define USB_ENDPOINT_DIR_OUT       0
#define USB_ENDPOINT_DIR_IN        1

    uint8_t  size;                        ///< 8, 16, 32 or 64 in bytes for endpoint buffer.
#define USB_ENDPOINT_SIZE_8_BYTES  0
#define USB_ENDPOINT_SIZE_16_BYTES 1
#define USB_ENDPOINT_SIZE_32_BYTES 2
#define USB_ENDPOINT_SIZE_64_BYTES 3

    uint8_t  banks;                       ///< USB_ENDPOINT_BANKS_* for single or double 
#define USB_ENDPOINT_BANKS_1       0
#define USB_ENDPOINT_BANKS_2       1

    volatile uint8_t tx_state;
#define EP_TX_STATE_IDLE           0
#define EP_TX_STATE_DATA           1
#define EP_TX_STATE_ZERO           2
#define EP_TX_STATE_WAIT           3

    char *buffer;                         ///< Endpoint transmit buffer pointer for in-flight
                                          ///  data transfer.
    int xmit_left;                        ///< Number of bytes still waiting to send

    uint8_t *recv_buf;                    ///< SRAM receive buffer pointer
    uint16_t recv_size;                   ///< SRAM buffer length
    uint16_t read_bytes;                  ///< Bytes in buffer that have been read

    /// Called when a SETUP request is received.
    void    (*rx_setup)(usb_endpoint_entry_t *ep);
    /// Called when a OUT request is recevived.
    void    (*rx_out)(usb_endpoint_entry_t *ep);
    /// Called when a transmit buffer is empty.
    /// and ready to accept more data.
    void    (*tx_in)(usb_endpoint_entry_t *ep);
    /// Called when the device wakes from suspend.
    void    (*wakeup)(usb_endpoint_entry_t *ep);
    /// Called when requested to suspend.
    void    (*sleep)(usb_endpoint_entry_t *ep);
    /// Called when a USB reset is received.
    void    (*reset)(usb_endpoint_entry_t *ep);
};


//
// General endpoint management from usb.c
//
void usb_endpoint_register(usb_endpoint_entry_t *endpoint);
int  usb_endpoint_buffer_read(usb_endpoint_entry_t *ep, char *buffer, int maxlen);
void usb_endpoint_buffer_write(usb_endpoint_entry_t *ep, char *buffer, int len);

//
// Control read/write functions from usb_control.c
//
int usb_txbuffer_start(usb_endpoint_entry_t *ep, char *buffer, int len);
int usb_txbuffer_done(usb_endpoint_entry_t *ep);

//
// Address and interrupt control functions
//
void usb_address_set(uint8_t addr);
void usb_address_enable(void);
void usb_address_disable(void);
void usb_endpoint_rxstp_enable(usb_endpoint_entry_t *ep);
void usb_endpoint_rxout_enable(usb_endpoint_entry_t *ep);
void usb_endpoint_configure_all(void);
void usb_attach(void);
void usb_detach(void);

//
// Initialization
//
void usb_init(void);
void usb_fini(void);

//
// From usb_control0
//
int usb_control0_init(char *desc, uint32_t desc_len,
                      char *config, uint32_t config_len,
                      usb_desc_string_t **str_desc);

typedef void (rx_vendor_setup_t)(usb_endpoint_entry_t *, usb_request_t *);
typedef uint16_t (rx_vendor_out_t)(usb_endpoint_entry_t *);
void usb_control0_vendor_register(rx_vendor_setup_t *, rx_vendor_out_t *);

#endif /* __USB_H__ */
