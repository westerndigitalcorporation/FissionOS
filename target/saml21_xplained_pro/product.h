/*
 * product.h
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


#ifndef __PRODUCT_H__
#define __PRODUCT_H__


// We don't have a VID/PID so use something that isn't valid.
// Unknown is already listed in linux as 0x0011
#define MANUFACTURER                             { 0x11, 0x00 }
#define PRODUCT                                  { 0x01, 0x01 }
#define VERSION                                  { 0x01, 0x00 }


/*
 *  USB transfer types
 */

/*
 *  bRequest
 */
#define USB_VENDOR_REQUEST_BOOTLOADER            0x01   // OUT
#define USB_VENDOR_REQUEST_RESET                 0x02   // OUT
#define USB_VENDOR_REQUEST_LED                   0x03   // OUT
#define USB_VENDOR_REQUEST_MODE                  0x04   // IN
#define USB_VENDOR_REQUEST_FLASH                 0x05   // OUT
#define USB_VENDOR_REQUEST_ADDR_LONG             0x06   // OUT
#define USB_VENDOR_REQUEST_ADDR_SHORT            0x07   // OUT
#define USB_VENDOR_REQUEST_PAN                   0x08   // OUT
#define USB_VENDOR_REQUEST_CHANNEL               0x09   // OUT
#define USB_VENDOR_REQUEST_CONFIG_WRITE          0x0a   // OUT
#define USB_VENDOR_REQUEST_POWER                 0x0b   // OUT

typedef struct usb_vendor_mode
{
        uint8_t mode;
#define USB_VENDOR_MODE_APPLICATION              0x0
#define USB_VENDOR_MODE_BOOTLOADER               0x1
} __attribute__ ((packed)) usb_vendor_mode_t;



#endif /* __PRODUCT_H__ */
