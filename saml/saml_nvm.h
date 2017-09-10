/*
 * saml_nvm.h
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


#ifndef __SAML_NVM_H__
#define __SAML_NVM_H__

#include "console.h"

#if defined(__AT91SAML21__) || defined(__AT91SAMD20__)

typedef struct nvmctrl
{
    uint16_t ctrla;
#define NVMCTRL_CTRLA_CMD_ER                     (0x02 << 0)
#define NVMCTRL_CTRLA_CMD_WP                     (0x04 << 0)
#define NVMCTRL_CTRLA_CMD_EAR                    (0x05 << 0)
#define NVMCTRL_CTRLA_CMD_WAP                    (0x06 << 0)
#define NVMCTRL_CTRLA_CMD_WL                     (0x0f << 0)
#define NVMCTRL_CTRLA_CMD_RWWEEER                (0x1a << 0)
#define NVMCTRL_CTRLA_CMD_RWWEEWP                (0x1c << 0)
#define NVMCTRL_CTRLA_CMD_LR                     (0x40 << 0)
#define NVMCTRL_CTRLA_CMD_UR                     (0x41 << 0)
#define NVMCTRL_CTRLA_CMD_SPRM                   (0x42 << 0)
#define NVMCTRL_CTRLA_CMD_CPRM                   (0x43 << 0)
#define NVMCTRL_CTRLA_CMD_PBC                    (0x44 << 0)
#define NVMCTRL_CTRLA_CMD_SSB                    (0x45 << 0)
#define NVMCTRL_CTRLA_CMD_INVALL                 (0x46 << 0)
#define NVMCTRL_CTRLA_CMDEX                      (0xa5 << 8)
    uint16_t resvd_0x02;
    uint32_t ctrlb;
#define NVMCTRL_CTRLB_RWS(val)                   ((val & 0xf) << 1)
#define NVMCTRL_CTRLB_MANW                       (1 << 7)
#define NVMCTRL_CTRLB_SLEEPPRM_WAKEUPACCESS      (0x0 << 8)
#define NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT     (0x1 << 8)
#define NVMCTRL_CTRLB_SLEEPPRM_DISABLED          (0x3 << 8)
#define NVMCTRL_CTRLB_READMODE_NO_MISS_PENALTY   (0x0 << 16)
#define NVMCTRL_CTRLB_READMODE_LOW_POWER         (0x1 << 16)
#define NVMCTRL_CTRLB_READMODE_DETERMINISTIC     (0x2 << 16)
#define NVMCTRL_CTRLB_CACHEDIS                   (1 << 18)
    uint32_t param;
#define NVMCTRL_PARAM_NVMP_GET(reg)              ((reg >> 0) & 0xffff)
#define NVMCTRL_PARAM_PSZ_GET(reg)               ((1 << ((reg >> 16) & 0x7)) << 3)
#define NVMCTRL_PARAM_RWWEEP_GET(reg)            ((reg >> 20) & 0xfff)
    uint8_t  intenclr;
#define NVMCTRL_INTENCLR_READY                   (1 << 0)
#define NVMCTRL_INTENCLR_ERROR                   (1 << 1)
    uint8_t  resvd_0x0d[3];
    uint8_t  intenset;
#define NVMCTRL_INTENSET_READY                   (1 << 0)
#define NVMCTRL_INTENSET_ERROR                   (1 << 1)
    uint8_t  resvd_0x11[3];
    uint8_t  intflag;
#define NVMCTRL_INTFLAG_READY                    (1 << 0)
#define NVMCTRL_INTFLAG_ERROR                    (1 << 1)
    uint8_t  resvd_0x15[3];
    uint16_t status;
#define NVMCTRL_STATUS_PRM                       (1 << 0)
#define NVMCTRL_STATUS_LOAD                      (1 << 1)
#define NVMCTRL_STATUS_PROGE                     (1 << 2)
#define NVMCTRL_STATUS_LOCKE                     (1 << 3)
#define NVMCTRL_STATUS_NVME                      (1 << 4)
#define NVMCTRL_STATUS_SB                        (1 << 8)
    uint16_t resvd_0x1a;
    uint32_t addr;
#define NVMCTRL_ADDR_ADDR(val)                   ((val & 0x3fffff) << 0)
    uint16_t lock;
} __attribute__((packed)) nvmctrl_t;


#define NVMCTRL                                  ((volatile nvmctrl_t *)0x41004000)


typedef struct nvm_user
{
    uint64_t row;
#define NVM_USER_BOOTPROT_GET(reg)               (((reg) >> 0) & 0x7)
#define NVM_USER_BOOTPROT_SET(reg, val)          ((reg & ~(0x7 << 0)) | ((val & 0x7) << 0))
#define NVM_USER_BOOTPROT_NONE                   (7 << 0)
#define NVM_USER_BOOTPROT_512                    (6 << 0)
#define NVM_USER_BOOTPROT_1024                   (5 << 0)
#define NVM_USER_BOOTPROT_2048                   (4 << 0)
#define NVM_USER_BOOTPROT_4096                   (3<< 0)
#define NVM_USER_BOOTPROT_8192                   (2 << 0)
#define NVM_USER_BOOTPROT_16384                  (1 << 0)
#define NVM_USER_BOOTPROT_32768                  (0 << 0)
#define NVM_USER_EEPROM_GET(reg)                 (((reg) >> 4) & 0x7)
#define NVM_USER_EEPROM_SET(reg, val)            ((reg & ~(0x7 << 4)) | ((val & 0x7) << 4))
#define NVM_USER_EEPROM_NONE                     (7 << 4)
#define NVM_USER_EEPROM_256                      (6 << 4)
#define NVM_USER_EEPROM_512                      (5 << 4)
#define NVM_USER_EEPROM_1024                     (4 << 4)
#define NVM_USER_EEPROM_2048                     (3 << 4)
#define NVM_USER_EEPROM_4096                     (2 << 4)
#define NVM_USER_EEPROM_8192                     (1 << 4)
#define NVM_USER_EEPROM_16384                    (0 << 4)
#define NVM_USER_BOD33_LVL_MASK                  (0x3f << 8)
#define NVM_USER_BOD33_LVL(val)                  ((val & 0x3f) << 8)
#define NVM_USER_BOD33_DISABLE                   (1 << 14)
#define NVM_USER_BOD33_ACTION_MASK               (0x3 << 15)
#define NVM_USER_BOD33_ACTION(val)               ((val & 0x3) << 15)
#define NVM_USER_BOD12_LVL_MASK                  (0x3 << 15)
#define NVM_USER_BOD12_LVL(val)                  ((val & 0x3f) << 17)
#define NVM_USER_BOD12_DISABLE                   (1 << 23)
#define NVM_USER_BOD12_ACTION_MASK               (0x3 << 24)
#define NVM_USER_BOD12_ACTION(val)               ((val & 0x3) << 24)
#define NVM_USER_WDT_ENABLE                      (1 << 26)
#define NVM_USER_WDT_ALWAYS                      (1 << 27)
#define NVM_USER_WDT_PERIOD_MASK                 (0xf << 28)
#define NVM_USER_WDT_PERIOD(val)                 ((val & 0xf) << 28)
#define NVM_USER_WDT_WINDOW_MASK                 (0xf << 32)
#define NVM_USER_WDT_WINDOW(val)                 ((val & 0xf) << 32)
#define NVM_USER_WDT_EWOFFSET_MASK               (0xf << 36)
#define NVM_USER_WDT_EWOFFSET(val)               ((val & 0xf) << 36)
#define NVM_USER_WDT_WEN                         (1 << 40)
#define NVM_USER_BOD33_HYST                      (1 << 41)
#define NVM_USER_BOD12_HYST                      (1 << 42)
#define NVM_USER_LOCK_MASK                       (0xffff < 48)
#define NVM_USER_LOCK(val)                       ((val & 0xffff) < 48)
} __attribute__((packed)) nvm_user_t;

#endif /* defined(__AT91SAML21__) || defined(__AT91SAMD20__) */

#if defined(__AT91SAML21__) || defined(__AT91SAMD20__)
#define NVM_USER_ROW                             ((volatile nvm_user_t *)0x00804000)
#define NVM_SOFT_CALIB                           (*(volatile uint32_t *)0x00806020)

#define NVM_SOFT_CALIB_USB_TRANSN                ((NVM_SOFT_CALIB >> 13) & 0x1f)
#define NVM_SOFT_CALIB_USB_TRANSP                ((NVM_SOFT_CALIB >> 18) & 0x1f)
#define NVM_SOFT_CALIB_USB_TRIM                  ((NVM_SOFT_CALIB >> 26) & 0x3f)

// Erases happen on row granularity, programs on page granularity
#define NVM_PAGE_SIZE                            64
#define NVM_PAGE_MASK                            (NVM_PAGE_SIZE - 1)
#define NVM_PAGES_PER_ROW                        4
#define NVM_ROW_MASK                             ((NVM_PAGES_PER_ROW * NVM_PAGE_SIZE) - 1)

typedef struct nvm_serial
{
    uint8_t num[16];
} __attribute__ ((packed)) nvm_serial_t;


#define NVM_SERIAL                               ((volatile nvm_serial_t *)0x0080a00c)


#define CONSOLE_CMD_NVM                          \
    {                                            \
        .cmdstr = "nvm",                         \
        .callback = cmd_nvm,                     \
        .usage = "  nvm < show | bootprot <value> | eeprom <value> >\r\n",       \
        .help =                                  \
            "  NVM Configuration.\r\n" \
            "    show     : Show the current NVM protetion settings.\r\n" \
            "    bootprot : Set the bootloader protection size.\r\n" \
            "    eeprom   : Set the EEPROM area size.\r\n" \
    }

#endif /* defined(__AT91SAML21__) */

#if defined(__ATSAMD53__)

typedef struct nvmctrl
{
    uint16_t ctrla;
#define NVMCTRL_CTRLA_AUTOWS                     (1 << 2)
#define NVMCTRL_CTRLA_SUSPEN                     (1 << 3)
#define NVMCTRL_CTRLA_WMODE(val)                 ((val & 0x3) << 4)
#define NVMCTRL_CTRLA_PRM(val)                   ((val & 0x3) << 6)
#define NVMCTRL_CTRLA_RWS(val)                   ((val & 0xf) << 8)
#define NVMCTRL_CTRLA_AHBNS0                     (1 << 12)
#define NVMCTRL_CTRLA_AHBNS1                     (1 << 13)
#define NVMCTRL_CTRLA_CACHEDIS0                  (1 << 14)
#define NVMCTRL_CTRLA_CACHEDIS1                  (1 << 15)
    uint16_t resvd_0x02;
    uint16_t ctrlb;
#define NVMCTRL_CTRLB_CMD(val)                   ((val & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_EP                     ((0x0 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_EB                     ((0x1 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_WP                     ((0x3 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_WQW                    ((0x4 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_SWRST                  ((0x10 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_LR                     ((0x11 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_UR                     ((0x12 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_SPRM                   ((0x13 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_CPRM                   ((0x14 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_PBC                    ((0x15 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_SSB                    ((0x16 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_BKSWRST                ((0x17 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_CELCK                  ((0x18 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_CEULK                  ((0x19 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_SBPDIS                 ((0x1a & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_CBPDIS                 ((0x1b & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_ASEES0                 ((0x30 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_ASEES1                 ((0x31 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_SEERALOC               ((0x32 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_SEEFLUSH               ((0x33 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_LSEE                   ((0x34 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_USEE                   ((0x35 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_LSEER                  ((0x36 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMD_USEER                  ((0x37 & 0x7f) << 0)
#define NVMCTRL_CTRLB_CMDEX                      (0xa5 << 8)
    uint16_t resvd_0x06;
    uint32_t param;
#define NVMCTRL_PARAM_NVMP_MASK                  (0xffff)
#define NVMCTRL_PARAM_NVMP_SHIFT                 (0)
#define NVMCTRL_PARAM_PSZ_MASK                   (0x7)
#define NVMCTRL_PARAM_PSZ_SHIFT                  (16)
#define NVMCTRL_PARAM_SEE                        (1 << 31)
    uint16_t intenclr;
#define NVMCTRL_INTENCLR_DONE                    (1 << 0)
#define NVMCTRL_INTENCLR_ADDRE                   (1 << 1)
#define NVMCTRL_INTENCLR_PROGE                   (1 << 2)
#define NVMCTRL_INTENCLR_LOCKE                   (1 << 3) 
#define NVMCTRL_INTENCLR_ECCSE                   (1 << 4)
#define NVMCTRL_INTENCLR_ECCDE                   (1 << 5)
#define NVMCTRL_INTENCLR_NVME                    (1 << 6)
#define NVMCTRL_INTENCLR_SUSP                    (1 << 7)
#define NVMCTRL_INTENCLR_SEESFULL                (1 << 8)
#define NVMCTRL_INTENCLR_SEESOVF                 (1 << 9)
#define NVMCTRL_INTENCLR_SEEWRC                  (1 << 10)
    uint16_t intenset;
#define NVMCTRL_INTENSET_DONE                    (1 << 0)
#define NVMCTRL_INTENSET_ADDRE                   (1 << 1)
#define NVMCTRL_INTENSET_PROGE                   (1 << 2)
#define NVMCTRL_INTENSET_LOCKE                   (1 << 3) 
#define NVMCTRL_INTENSET_ECCSE                   (1 << 4)
#define NVMCTRL_INTENSET_ECCDE                   (1 << 5)
#define NVMCTRL_INTENSET_NVME                    (1 << 6)
#define NVMCTRL_INTENSET_SUSP                    (1 << 7)
#define NVMCTRL_INTENSET_SEESFULL                (1 << 8)
#define NVMCTRL_INTENSET_SEESOVF                 (1 << 9)
#define NVMCTRL_INTENSET_SEEWRC                  (1 << 10)
    uint16_t intflag;
#define NVMCTRL_INTFLAG_DONE                     (1 << 0)
#define NVMCTRL_INTFLAG_ADDRE                    (1 << 1)
#define NVMCTRL_INTFLAG_PROGE                    (1 << 2)
#define NVMCTRL_INTFLAG_LOCKE                    (1 << 3) 
#define NVMCTRL_INTFLAG_ECCSE                    (1 << 4)
#define NVMCTRL_INTFLAG_ECCDE                    (1 << 5)
#define NVMCTRL_INTFLAG_NVME                     (1 << 6)
#define NVMCTRL_INTFLAG_SUSP                     (1 << 7)
#define NVMCTRL_INTFLAG_SEESFULL                 (1 << 8)
#define NVMCTRL_INTFLAG_SEESOVF                  (1 << 9)
#define NVMCTRL_INTFLAG_SEEWRC                   (1 << 10)
    uint16_t status;
#define NVMCTRL_STATUS_READY                     (1 << 0)
#define NVMCTRL_STATUS_PRM                       (1 << 1)
#define NVMCTRL_STATUS_LOAD                      (1 << 2)
#define NVMCTRL_STATUS_SUSP                      (1 << 3)
#define NVMCTRL_STATUS_AFIRST                    (1 << 4)
#define NVMCTRL_STATUS_BPDIS                     (1 << 5)
#define NVMCTRL_STATUS_BOOTPROT(val)             ((val & 0xf) << 8)
    uint32_t addr;
    uint32_t runlock;
    uint32_t pbldatan0;
    uint32_t pbldatan1;
    uint32_t eccerr;
#define NVMCTRL_ECCERR_ADDR(val)                 ((val & 0x00ffffff) << 0)
#define NVMCTRL_ECCERR_TYPEL(val)                ((val & 0x3) << 4)
#define NVMCTRL_ECCERR_TYPEH(val)                ((val & 0x3) << 6)
    uint8_t  dbgctrl;
#define NVMCTRL_DBGCTRL_ECCDIS                   (1 << 0)
#define NVMCTRL_DBGCTRL_ECCELOG                  (1 << 1)
    uint8_t  resvd_0x29;
    uint8_t  seecfg;
    uint8_t  resvd_0x2b;
    uint32_t seestat;
} __attribute__ ((packed)) nvmctrl_t;

#define NVMCTRL                                  ((volatile nvmctrl_t *)0x41004000)

#define NVM_USER_ROW                             ((volatile nvm_user_t *)0x00804000)
#define NVM_SOFT_CALIB_ADC                       (*(volatile uint32_t *)0x00800080)
#define NVM_SOFT_CALIB_USB                       (*(volatile uint32_t *)0x00800084)

#define NVM_SOFT_CALIB_USB_TRANSN                ((NVM_SOFT_CALIB_USB >> 0) & 0x1f)
#define NVM_SOFT_CALIB_USB_TRANSP                ((NVM_SOFT_CALIB_USB >> 5) & 0x1f)
#define NVM_SOFT_CALIB_USB_TRIM                  ((NVM_SOFT_CALIB_USB >> 10) & 0x07)

#define NVM_SOFT_CALIB_ADC0_BIASCOMP             ((NVM_SOFT_CALIB_ADC >> 2) & 0x7)
#define NVM_SOFT_CALIB_ADC0_BIASREFBUF           ((NVM_SOFT_CALIB_ADC >> 5) & 0x7)
#define NVM_SOFT_CALIB_ADC0_BIASR2R              ((NVM_SOFT_CALIB_ADC >> 8) & 0x7)

#define NVM_SOFT_CALIB_ADC1_BIASCOMP             ((NVM_SOFT_CALIB_ADC >> 16) & 0x7)
#define NVM_SOFT_CALIB_ADC1_BIASREFBUF           ((NVM_SOFT_CALIB_ADC >> 19) & 0x7)
#define NVM_SOFT_CALIB_ADC1_BIASR2R              ((NVM_SOFT_CALIB_ADC >> 22) & 0x7)

// Erases happen on block granularity, programs on page granularity
#define NVM_PAGE_SIZE                            512
#define NVM_PAGE_MASK                            (NVM_PAGE_SIZE - 1)
#define NVM_ERASE_SIZE                           (16 * NVM_PAGE_SIZE)
#define NVM_ERASE_MASK                           (NVM_ERASE_SIZE - 1)
#define NVM_PROTECTION_REGIONS                   32

#define CONSOLE_CMD_NVM                          \
    {                                            \
        .cmdstr = "nvm",                         \
        .callback = cmd_nvm,                     \
        .usage = "  nvm < show >\r\n",       \
        .help =                                  \
            "  NVM Configuration.\r\n" \
            "    swbk     : Switch active NVM banks.\r\n" \
            "    show     : Show the current NVM protetion settings.\r\n", \
    }

void nvm_cache_disable(void);
void nvm_cache_enable(void);

uint32_t nvm_bank_offset(void);
int nvm_active_bank(void);
void nvm_switch_bank(void);
uint32_t nvm_crc32(uint32_t addr, uint32_t len);

#endif /* #if defined(__ATSAMD53__) */

int cmd_nvm(console_t *console, int argc, char *argv[]);

// data and len must be multiples of 32-bit words
void nvm_write(uint32_t addr, uint8_t *data, uint32_t len);


#endif /* __SAML_NVM_H__ */
