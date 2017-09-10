/*
 * usb_messages.h
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


#ifndef __USB_MESSAGES_H__
#define __USB_MESSAGES_H__

//
// Standard USB message definitions
//
typedef struct usb_request
{
    uint8_t  request_type;
#define USB_REQ_DIR_MASK                         0x80
#define USB_REQ_DIR_DEV_TO_HOST                  (1 << 7)
#define USB_REQ_DIR_HOST_TO_DEV                  (0 << 7)

#define USB_REQ_TYPE_MASK                        0x60
#define USB_REQ_TYPE_STANDARD                    (0 << 5)
#define USB_REQ_TYPE_CLASS                       (1 << 5)
#define USB_REQ_TYPE_VENDOR                      (2 << 5)

#define USB_REQ_RECP_MASK                        0x1f
#define USB_REQ_RECP_STANDARD                    0
#define USB_REQ_RECP_INTERFACE                   1
#define USB_REQ_RECP_ENDPOINT                    2

    uint8_t  request;
#define USB_REQ_GET_STATUS                       0
#define USB_REQ_CLEAR_FEATURE                    1
#define USB_REQ_SET_FEATURE                      3
#define USB_REQ_ADDRESS                          5
#define USB_REQ_GET_DESCRIPTOR                   6
#define USB_REQ_SET_DESCRIPTOR                   7
#define USB_REQ_GET_CONFIG                       8
#define USB_REQ_SET_CONFIG                       9
#define USB_REQ_GET_INTERFACE                    10
#define USB_REQ_SET_INTERFACE                    11
#define USB_REQ_SYNCH_FRAME                      12

#define USB_REQ_SET_LINE_CODING                  32
#define USB_REQ_SET_CONTROL_LINE_STATE           34

    uint8_t value[2];
// Second byte can be the desc type
#define USB_DESC_DEVICE         1
#define USB_DESC_CONFIG         2
#define USB_DESC_STRING         3
    uint16_t index;
    uint16_t length;
} __attribute__((packed)) usb_request_t;

typedef struct usb_desc_device
{
    uint8_t    length;
    uint8_t    type;
#define USB_DESC_TYPE_DEVICE         1
#define USB_DESC_TYPE_HID            0x21
#define USB_DESC_TYPE_HID_REPORT     0x22
#define USB_DESC_TYPE_HID_PHYS       0x23
    uint8_t    version[2];
    uint8_t    class;
    uint8_t    subclass;
    uint8_t    protocol;
    uint8_t    max_packet;
    uint8_t    vendor_id[2];
    uint8_t    device_id[2];
    uint8_t    device_version[2];

    uint8_t    vendor_str_index;
    uint8_t    product_str_index;
    uint8_t    serial_str_index;

    uint8_t    num_configs;
} __attribute__((packed)) usb_desc_device_t;

typedef struct usb_desc_config
{
    uint8_t  length;
    uint8_t  type;
#define USB_DESC_TYPE_CONFIG         2
    uint16_t total_length;
    uint8_t  num_interfaces;
    uint8_t  config_value;
    uint8_t  string_index;
    uint8_t  attributes;
    uint8_t  max_power_ma;
    // Additional interfaces would follow
} __attribute__((packed)) usb_desc_config_t;

typedef struct usb_desc_string
{
    uint8_t    length;
    uint8_t    type;
#define USB_DESC_TYPE_STRING         3
    uint8_t    buffer[];  // unicode string
} __attribute__((packed)) usb_desc_string_t;

typedef struct usb_desc_interface
{
    uint8_t  length;
    uint8_t  type;
#define USB_DESC_TYPE_INTERFACE      4
    uint8_t  number;
    uint8_t  alt_setting;
    uint8_t  num_endpoints;
    uint8_t  class;
    uint8_t  sub_class;
    uint8_t  protocol;
    uint8_t  string_index;
} __attribute__((packed)) usb_desc_interface_t;

#define USB_DESC_TYPE_CS_INTERFACE   0x24
typedef struct usb_desc_cdc_header
{
    uint8_t length;
    uint8_t type;
    uint8_t sub_type;
#define USB_DESC_CDC_HEADER_SUBTYPE  0x00
    uint16_t bcd_cdc;                     // 0x10 0x01
} __attribute__((packed)) usb_desc_cdc_header_t;

typedef struct usb_desc_cdc_acm
{
    uint8_t length;
    uint8_t type;
    uint8_t sub_type;
#define USB_DESC_CDC_ACM_SUBTYPE     0x02  // ACM
    uint8_t capabilities;
} __attribute__((packed)) usb_desc_cdc_acm_t;

typedef struct usb_desc_cdc_union
{
    uint8_t length;
    uint8_t type;
    uint8_t sub_type;
#define USB_DESC_CDC_UNION_SUBTYPE   0x06
    uint8_t master_interface;
    uint8_t slave_interface;
} __attribute__((packed)) usb_desc_cdc_union_t;

typedef struct usb_desc_endpoint
{
    uint8_t  length;
    uint8_t  type;
#define USB_DESC_TYPE_ENDPOINT       5
    uint8_t  ep_addr;
#define USB_EP_ADDR_OUT              0x00
#define USB_EP_ADDR_IN               0x80
    uint8_t  attrs;
#define USB_DESC_EP_ATTR_CONTROL     0
#define USB_DESC_EP_ATTR_ISO         1
#define USB_DESC_EP_ATTR_BULK        2
#define USB_DESC_EP_ATTR_INT         3
    uint16_t max_pkt_size;
    uint8_t  interval;
} __attribute__((packed)) usb_desc_endpoint_t;

#endif /* __USB_MESSAGES_H__ */
