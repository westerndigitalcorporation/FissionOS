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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <libusb-1.0/libusb.h>

#include "fwheader.h"

#include "product.h"


#define FLASH_BLOCK_SIZE                          4096


typedef int (callback_t)(char *arg);

static struct libusb_device_handle *usbdev = NULL;
static struct libusb_context *usbcontext;

char *short_opts = "a:ru:i";

void usage(char *argv0)
{
    printf("\n");
    printf("%s: [command]\n", argv0);
    printf("\n");
    printf("Local USB Commands (no ipv6 address):\n");
    printf("    -i              Device Information\n");
    printf("    -r              Reset Device\n");
    printf("    -u <filename>   Upload firmware file\n");
    printf("    -h              Help\n");
    printf("\n");
}

/*
 * USB Functions
 */

struct libusb_device_handle *dev_open(void)
{
    struct libusb_device_handle *dev;

    if (libusb_init(&usbcontext))
    {
        fprintf(stderr, "libusb_init failed\n");
        return NULL;
    }

    dev = libusb_open_device_with_vid_pid(usbcontext, 0x0011, 0x0101);
    if (!dev)
    {
        fprintf(stderr, "No device found\n");
        libusb_exit(usbcontext);
        return NULL;
    }

    libusb_detach_kernel_driver(dev, 0);
    if (libusb_claim_interface(dev, 0))
    {
        libusb_close(dev);
        libusb_exit(usbcontext);
        return NULL;
    }

    return dev;
}

void dev_close(struct libusb_device_handle *dev)
{
    libusb_attach_kernel_driver(dev, 0);
    libusb_close(dev);
    libusb_exit(usbcontext);
}

int get_device(char *arg)
{
	device_info_t device_info;

    if (libusb_control_transfer(usbdev,
                                LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
                                LIBUSB_RECIPIENT_DEVICE,
                                USB_VENDOR_REQUEST_INFO, 0, 0,
                                (uint8_t *)&device_info, sizeof(device_info), 0) != sizeof(device_info))
    {
        fprintf(stderr, "libusb_control_transfer failed\n");
        return -1;
    }

    printf("Bank: %d\n", device_info.bank);
    printf("Size: %dKb\n", device_info.size / 1024);

    return 0;
}


int reset_device(char *arg)
{
    if (libusb_control_transfer(usbdev,
                                LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                LIBUSB_RECIPIENT_DEVICE,
                                USB_VENDOR_REQUEST_RESET, 0, 0,
                                NULL, 0, 0) != 0)
    {
        fprintf(stderr, "libusb_control_transfer failed\n");
        return -1;
    }

    return 0;
}

int update(char *arg)
{
    char *filename = arg;
    FILE *f = fopen(filename, "r");
    uint32_t addr = 0;
    uint8_t buf[FLASH_BLOCK_SIZE];
    fwheader_t header;
    int n, ret;

    if (!f)
    {
        fprintf(stderr, "can't open file\n");
        return -1;
    }

    n = fread(&header, 1, sizeof(header), f);
    if (n != sizeof(header))
    {
        fprintf(stderr, "can't read file\n");
        fclose(f);
        return -1;
    }

    if (header.magic != FWHEADER_MAGIC)
    {
        fprintf(stderr, "Not a firmware file\n");
        fclose(f);
        return -1;
    }

    fseek(f, sizeof(header), SEEK_SET);

    n = fread(buf, 1, sizeof(buf), f);
    while (n > 0)
    {
        ret = libusb_control_transfer(usbdev,
                                      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                      LIBUSB_RECIPIENT_DEVICE,
                                      USB_VENDOR_REQUEST_FLASH,
                                      addr >> 16, addr & 0xffff,
                                      buf, n, 0);
        if (ret != n)
        {
            fprintf(stderr, "Firmware update failed %d %d\n", ret, n);
            fclose(f);

            return -1;
        }

        addr += n;
        n = fread(buf, 1, sizeof(buf), f);
    }

    if (n < 0)
    {
        fprintf(stderr, "File read error\n");
        fclose(f);

        return -1;
    }

    ret = libusb_control_transfer(usbdev,
                                  LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                  LIBUSB_RECIPIENT_DEVICE,
                                  USB_VENDOR_REQUEST_FLASH_DONE,
                                  0, 0,
                                  (uint8_t *)&header, sizeof(header), 0);

    if (ret != sizeof(header))
    {
        fprintf(stderr, "Firmware signature failed %d %d\n", ret, n);
        fclose(f);

        return -1;
    }

    fclose(f);

    return 0;
}

int usb_command(callback_t *callback, char *arg)
{
    int ret;

    usbdev = dev_open();
    if (!usbdev)
    {
        printf("Device not found\n");
        return 0;
    }

    ret = callback(arg); 

    dev_close(usbdev);

    return ret;
}

int main(int argc, char *argv[])
{
    callback_t *callback = NULL;
    char *addr = NULL;
    char *arg = NULL;
    int opt;

    while ((opt = getopt(argc, argv, short_opts)) != -1)
    {
        switch (opt)
        {
            case 'a':
                addr = optarg;
                break;

            case 'i':
                callback = get_device;
                break;

            case 'r':
                callback = reset_device;
                break;

            case 'u':
                callback = update;
                arg = optarg;
                break;

            case 'h':
            default:
                usage(argv[0]);
                return -1;
                break;
        }
    }

    if (!callback)
    {
        usage(argv[0]);
        return -1;
    }

    // Local commands
    if (!addr)
    {
        return usb_command(callback, arg);
    }

    return 0;
}


