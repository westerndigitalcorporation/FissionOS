/*
 * smbus_master.h
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

#ifndef __SMBUS_MASTER_H__
#define __SMBUS_MASTER_H__

typedef struct smbus_udid
{
    uint8_t  capabilities;
#define SMBUS_UDID_CAP_FIXED                     (0x0 << 6)
#define SMBUS_UDID_CAP_DYNAMIC_PERSIST           (0x1 << 6)
#define SMBUS_UDID_CAP_DYNAMIC_VOLATILE          (0x2 << 6)
#define SMBUS_UDID_CAP_RANDOM                    (0x3 << 6)
#define SMBUS_UDID_CAP_PEC_SUPPORTED             (1 << 0)
    uint8_t  version;
#define SMBUS_UDID_VERSION_UDID1                 (0x1 << 3)
#define SMBUS_UDID_VERSION_SILICON(val)          (((val) & 0x7) << 0)
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t interface;
#define SMBUS_UDID_INTERFACE_ZONE                (1 << 7)
#define SMBUS_UDID_INTERFACE_IPMI                (1 << 6)
#define SMBUS_UDID_INTERFACE_ASF                 (1 << 5)
#define SMBUS_UDID_INTERFACE_OEM                 (1 << 4)
#define SMBUS_UDID_INTERFACE_VERSION_1_0         (0x0 << 0)
#define SMBUS_UDID_INTERFACE_VERSION_1_1         (0x1 << 0)
#define SMBUS_UDID_INTERFACE_VERSION_2_0         (0x4 << 0)
#define SMBUS_UDID_INTERFACE_VERSION_3_0         (0x5 << 0)
    uint16_t sub_vendor_id;
    uint16_t sub_device_id;
    uint32_t vendor_specific_id;
} __attribute__((packed)) smbus_udid_t;


/*
 * SMBus ARP Address and Command definitions
 */
#define SMBUS_ARP_ADDRESS                        0xc2

#define SMBUS_ARP_COMMAND_PREPARE                0x1
#define SMBUS_ARP_COMMAND_RESET                  0x2
#define SMBUS_ARP_COMMAND_GET_UDID               0x3
#define SMBUS_ARP_COMMAND_ASSIGN_ADDR            0x4


// SMBUS ARP Packet Formats
//
// Prepare to ARP
// ADDR/W 0xc2 | COMMAND 0x1 | PEC
//
// Reset Device (General)
// ADDR/W 0xc2 | COMMAND 0x2 | PEC
//
// Reset Device (Targetted)
// ADDR/W 0xc2 | DEVICE SLAVE ADDR | PEC
//
// Get UDID (General)
// ADDR/W 0xc2 | COMMAND 0x3 | *Restart* | ADDR/R | BYTE COUNT (17) | ... 16 ID BYTES ... | DEVICE SLAVE ADDR | PEC
//
// Get UDID (Targetted)
// ADDR/W 0xc2 | DEVICE SLAVE ADDR | *Restart* | ADDR/R | BYTE COUNT (17) | ... 16 ID BYTES ... | DEVICE SLAVE ADDR | PEC
//
// Assign Address
// ADDR/W 0xc2 | COMMAND 0x4 | BYTE_COUNT (17) | ... 16 ID BYTES ... | ASSIGNED ADDRESS | PEC
//
//
// Notify ARP Master
// MASTER ADDR/W 0x16 | DEVICE ADDR | DATA LOW | DATA HIGH
//


#define CONSOLE_CMD_SMBUS                        \
    {                                            \
        .cmdstr = "smbus",                       \
        .callback = cmd_smbus,                   \
        .usage = "  smbus <list | <devicenum> <addr> <read len> [write data ...]>\r\n",       \
        .help =                                  \
            "  Read and write data on a given I2c interface.\r\n"  \
            "    list       : Show available i2c devices.\r\n" \
            "    device     : I2c interface number.\r\n" \
            "    addr       : I2c slave address.\r\n" \
            "    read len   : Length in bytes to read from slave.\r\n" \
            "    write data : Bytes to write separated by spaces.\r\n", \
    }

int cmd_smbus(uart_drv_t *uart, int argc, char *argv[]);

#endif /* __SMBUS_MASTER_H__ */
