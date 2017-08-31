/*
 * sam4_gpio.h
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


#ifndef __SAM4S_GPIO_H__
#define __SAM4S_GPIO_H__

typedef struct gpio_regs
{
    uint32_t per;
    uint32_t pdr;
    uint32_t psr;
    uint32_t resvd_0x0c;
    uint32_t oer;
    uint32_t odr;
    uint32_t osr;
    uint32_t resvd_0x1c;
    uint32_t ifer;
    uint32_t ifdr;
    uint32_t ifsr;
    uint32_t resvd_0x2c;
    uint32_t sodr;
    uint32_t codr;
    uint32_t odsr;
    uint32_t pdsr;
    uint32_t ier;
    uint32_t idr;
    uint32_t imr;
    uint32_t isr;
    uint32_t mder;
    uint32_t mddr;
    uint32_t mdsr;
    uint32_t resvd_0x5c;
    uint32_t pudr;
    uint32_t puer;
    uint32_t pusr;
    uint32_t resvd_0x6c;
    uint32_t abcdsr1;
    uint32_t abcdsr2;
    uint32_t resvd_0x78[2];
    uint32_t ifscdr;
    uint32_t ifscer;
    uint32_t ifscsr;
    uint32_t scdr;
    uint32_t ppddr;
    uint32_t ppder;
    uint32_t ppdsr;
    uint32_t resvd_0x9c;
    uint32_t ower;
    uint32_t owdr;
    uint32_t owsr;
    uint32_t resvd_0xac;
    uint32_t aimer;
    uint32_t aimdr;
    uint32_t aimmr;
    uint32_t resvd_0xbc;
    uint32_t esr;
    uint32_t lsr;
    uint32_t elsr;
    uint32_t resvd_0xcc;
    uint32_t fellsr;
    uint32_t rehlsr;
    uint32_t frlhsr;
    uint32_t resvd_0xdc;
    uint32_t locksr;
    uint32_t wpmr;
    uint32_t wpsr;
    uint32_t resvd_0xec[5];
    uint32_t schmitt;
    uint32_t resvd_0x104[19];
    uint32_t pcmr;
    uint32_t pcier;
    uint32_t pcidr;
    uint32_t pcimr;
    uint32_t pcisr;
    uint32_t pcrhr;
} __attribute__ ((packed)) gpio_regs_t;

#define GPIO_ADDR                                0x400e0e00
#define GPIOA                                    ((volatile gpio_regs_t *)(GPIO_ADDR + 0x0000))
#define GPIOB                                    ((volatile gpio_regs_t *)(GPIO_ADDR + 0x0200))
#define GPIOC                                    ((volatile gpio_regs_t *)(GPIO_ADDR + 0x0400))
#define GPIOD                                    ((volatile gpio_regs_t *)(GPIO_ADDR + 0x0600))

#define GPIO_PERIPHERAL_A                        0
#define GPIO_PERIPHERAL_B                        1
#define GPIO_PERIPHERAL_C                        2
#define GPIO_PERIPHERAL_SET(gpio, pin, periph)                                                       \
                                                 do {                                                \
                                                     gpio->abcdsr1 &= ~(1 << pin);                   \
                                                     gpio->abcdsr2 &= ~(1 << pin);                   \
                                                     gpio->abcdsr1 |= (periph & 0x1) ? 1 << pin : 0; \
                                                     gpio->abcdsr2 |= (periph & 0x2) ? 1 << pin : 0; \
                                                 } while(0)

#define GPIO_DISABLE(gpio, pin)                  gpio->pdr = (1 << (pin))
#define GPIO_ENABLE(gpio, pin)                   gpio->per = (1 << (pin))
#define GPIO_PULLUP_ENABLE(gpio, pin)            gpio->puer = (1 << (pin))
#define GPIO_PULLUP_DISABLE(gpio, pin)           gpio->pudr = (1 << (pin))
#define GPIO_PULLDOWN_ENABLE(gpio, pin)          gpio->ppder = (1 << (pin))
#define GPIO_PULLDOWN_DISABLE(gpio, pin)         gpio->ppddr = (1 << (pin))
#define GPIO_MULTIDRIVE_ENABLE(gpio, pin)        gpio->mder = (1 << (pin))
#define GPIO_MULTIDRIVE_DISABLE(gpio, pin)       gpio->mddr = (1 << (pin))

#define GPIO_SET_INPUT(gpio, pin)                gpio->odr = (1 << (pin))
#define GPIO_SET_OUTPUT(gpio, pin)               gpio->oer = (1 << (pin))
#define GPIO_SET_HIGH(gpio, pin)                 gpio->sodr = (1 << (pin))
#define GPIO_SET_LOW(gpio, pin)                  gpio->codr = (1 << (pin))
#define GPIO_GET_INPUT(gpio, pin)                (gpio->pdsr & (1 << (pin)))


/* Special PIN remapping */
#define CCFG_SYSIO                               (*(volatile uint32_t *)0x400e0314)
#define CCFG_SYSIO_SYSIO4                        (1 << 4)
#define CCFG_SYSIO_SYSIO5                        (1 << 5)
#define CCFG_SYSIO_SYSIO6                        (1 << 6)
#define CCFG_SYSIO_SYSIO7                        (1 << 7)
#define CCFG_SYSIO_SYSI10                        (1 << 10)
#define CCFG_SYSIO_SYSI11                        (1 << 11)
#define CCFG_SYSIO_SYSI12                        (1 << 12)

static inline void jtag_disable(void)
{
    // Make sure PB4 and PB5 are mapped as GPIO/Peripheral
    CCFG_SYSIO |= CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5;
}

static inline void usb_disable(void)
{
    // Make sure PB11 and PB10 are mapped to GPIO
    CCFG_SYSIO |= CCFG_SYSIO_SYSI10 | CCFG_SYSIO_SYSI11;
}

// These functions assume LEDs are active low
#define LED_INIT(port, mask)                     do {                    \
                                                     port->sodr = mask;  \
                                                     port->oer = mask;   \
                                                     port->per = mask;   \
                                                 } while (0)
#define LED_ON(port, mask)                       port->codr = mask
#define LED_OFF(port, mask)                      port->sodr = mask

#endif   /* __SAM4S_GPIO_H__ */
