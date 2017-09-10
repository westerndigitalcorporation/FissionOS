/*
 * usb_config.h
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
 * Author: Jeremy Garff <je@jers.net>
 *
 */


#define USB_SETTLE_mS                            50

// String Descriptors referenced from the device descriptor
#define USB_STR_DESC(name, data, len)                         \
    usb_desc_string_t name =                                  \
    {                                                         \
        .length            = sizeof(usb_desc_string_t) +      \
                             len,                             \
        .type              = USB_DESC_TYPE_STRING,            \
        .buffer            = data                             \
    }

#define USB_MAX_ENDPOINT                         4

extern usb_desc_string_t *usb_str_desc[];
extern usb_desc_device_t usb_desc;
extern uint32_t usb_desc_len;

// Configuration Descriptor
typedef struct usb_desc_config_all {
    usb_desc_config_t    config1;
    usb_desc_interface_t interface1;
    usb_desc_cdc_header_t header_desc;
    usb_desc_cdc_acm_t acm_desc;
    usb_desc_cdc_union_t union_desc;
    usb_desc_endpoint_t  ep1;
    usb_desc_interface_t interface2;
    usb_desc_endpoint_t  ep2;
    usb_desc_endpoint_t  ep3;
} usb_desc_config_all_t;

extern usb_desc_config_all_t usb_config;
extern uint32_t usb_config_len;


