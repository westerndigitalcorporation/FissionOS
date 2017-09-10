/*
 * usb_config.c
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


#include <stdint.h>
#include <stddef.h>

#include "usb_messages.h"
#include "usb_config.h"

#include "product.h"


#define USB_STR_LANG        { 0x09, 0x04 }  // Little endian 0x0409, English
USB_STR_DESC(usb_str_lang, USB_STR_LANG, 2);

#define USB_STR_VENDOR      { 'T', 0, 'e', 0, 's', 0, 't', 0, }  // Unicode
USB_STR_DESC(usb_str_vendor, USB_STR_VENDOR, 8);

#define USB_STR_PRODUCT     { 'T', 0, 'e', 0, 's', 0, 't', 0 }
USB_STR_DESC(usb_str_product, USB_STR_PRODUCT, 8);

usb_desc_string_t *usb_str_desc[] = 
{
    &usb_str_lang,    // String Index 0, Supported Languages
    &usb_str_vendor,  // String Index 1
    &usb_str_product  // String Index 2
};


// Main device descriptor
usb_desc_device_t usb_desc = 
{
    .length                = sizeof(usb_desc_device_t),
    .type                  = USB_DESC_TYPE_DEVICE,
    .version               = { 0x01, 0x01 },
    .class                 = 2, // Communications
    .subclass              = 0,
    .protocol              = 0,
    .max_packet            = 64,
    .vendor_id             = MANUFACTURER,
    .device_id             = PRODUCT,
    .device_version        = VERSION,

    .vendor_str_index      = 1,
    .product_str_index     = 2,
    .serial_str_index      = 0,

    .num_configs           = 1
};
uint32_t usb_desc_len = sizeof(usb_desc);

usb_desc_config_all_t usb_config = 
{
    .config1 = 
    {
        .length            = sizeof(usb_desc_config_t),
        .type              = USB_DESC_TYPE_CONFIG,
        .total_length      = sizeof(usb_desc_config_all_t),
        .num_interfaces    = 2,
        .config_value      = 1,
        .string_index      = 0,
        .attributes        = 0x80,  // Must set to 1 if usb ver > 1.0
        .max_power_ma      = 25     // In ma divided by 2 = 50ma
    },

    .interface1 =
    {
        .length            = sizeof(usb_desc_interface_t),
        .type              = USB_DESC_TYPE_INTERFACE,
        .number            = 0,
        .alt_setting       = 0,
        .num_endpoints     = 1,
        .class             = 2, // Communications
        .sub_class         = 2, // Modem
        .protocol          = 1, // AT-commands (v.25ter)
        .string_index      = 0
    },

    .header_desc =
    {
        .length            = sizeof(usb_desc_cdc_header_t),
        .type              = USB_DESC_TYPE_CS_INTERFACE,
        .sub_type          = USB_DESC_CDC_HEADER_SUBTYPE,
        .bcd_cdc           = 0x0110,
    },

    .acm_desc =
    {
        .length            = sizeof(usb_desc_cdc_acm_t),
        .type              = USB_DESC_TYPE_CS_INTERFACE,
        .sub_type          = USB_DESC_CDC_ACM_SUBTYPE,
        .capabilities      = 0x0,
    },

    .union_desc =
    {
        .length            = sizeof(usb_desc_cdc_union_t),
        .type              = USB_DESC_TYPE_CS_INTERFACE,
        .sub_type          = USB_DESC_CDC_UNION_SUBTYPE,
        .master_interface  = 0,
        .slave_interface   = 1,
    },

    .ep1 =
    {
        .length            = sizeof(usb_desc_endpoint_t),
        .type              = USB_DESC_TYPE_ENDPOINT,
        .ep_addr           = USB_EP_ADDR_IN | 1,
        .attrs             = USB_DESC_EP_ATTR_INT,
        .max_pkt_size      = 8,
        .interval          = 32,
    },

    .interface2 =
    {
        .length            = sizeof(usb_desc_interface_t),
        .type              = USB_DESC_TYPE_INTERFACE,
        .number            = 1,
        .alt_setting       = 0,
        .num_endpoints     = 2,
        .class             = 0xa, // Communications
        .sub_class         = 0,
        .protocol          = 0,
        .string_index      = 0
    },

    .ep2 =
    {
        .length            = sizeof(usb_desc_endpoint_t),
        .type              = USB_DESC_TYPE_ENDPOINT,
        .ep_addr           = USB_EP_ADDR_IN | 2,
        .attrs             = USB_DESC_EP_ATTR_BULK,
        .max_pkt_size      = 64,
        .interval          = 0,
    },

    .ep3 =
    {
        .length            = sizeof(usb_desc_endpoint_t),
        .type              = USB_DESC_TYPE_ENDPOINT,
        .ep_addr           = USB_EP_ADDR_OUT | 2,
        .attrs             = USB_DESC_EP_ATTR_BULK,
        .max_pkt_size      = 64,
        .interval          = 0,
    },
};
uint32_t usb_config_len = sizeof(usb_config);


