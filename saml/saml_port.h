/*
 * saml_port.h
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


#ifndef __SAML_PORT_H__
#define __SAML_PORT_H__


typedef struct port
{
    uint32_t dir;
    uint32_t dirclr;
    uint32_t dirset;
    uint32_t dirtgl;
    uint32_t out;
    uint32_t outclr;
    uint32_t outset;
    uint32_t outtgl;
    uint32_t in;
    uint32_t ctrl;
    uint32_t wrconfig;
#define PORT_WRCONFIG_PINMASK(val)               ((val & 0xffff) << 0)
#define PORT_WRCONFIG_PMUXEN                     (1 << 16)
#define PORT_WRCONFIG_INEN                       (1 << 17)
#define PORT_WRCONFIG_PULLEN                     (1 << 18)
#define PORT_WRCONFIG_DRVSTR                     (1 << 22)
#define PORT_WRCONFIG_PMUX(val)                  ((val & 0xf) << 24)
#define PORT_WRCONFIG_WRPMUX                     (1 << 28)
#define PORT_WRCONFIG_WRPINCFG                   (1 << 30)
#define PORT_WRCONFIG_HWSEL                      (1 << 31)
    uint32_t evctrl;
#define PORT_EVCTRL_PID0(val)                    ((val & 0x1f) << 0)
#define PORT_EVCTRL_EVACT0(val)                  ((val & 0x3) << 5)
#define PORT_EVCTRL_PORTEI0                      (1 << 7)
#define PORT_EVCTRL_PID1(val)                    ((val & 0x1f) << 8)
#define PORT_EVCTRL_EVACT1(val)                  ((val & 0x3) << 13)
#define PORT_EVCTRL_PORTEI1                      (1 << 15)
#define PORT_EVCTRL_PID2(val)                    ((val & 0x1f) << 16)
#define PORT_EVCTRL_EVACT2(val)                  ((val & 0x3) << 21)
#define PORT_EVCTRL_PORTEI2                      (1 << 23)
#define PORT_EVCTRL_PID3(val)                    ((val & 0x1f) << 24)
#define PORT_EVCTRL_EVACT3(val)                  ((val & 0x3) << 29)
#define PORT_EVCTRL_PORTEI3                      (1 << 31)
    uint8_t pmux[16];
#define PORT_PMUX_PMUXE_MASK                     (0xf << 0)
#define PORT_PMUX_PMUXE(val)                     ((val & 0xf) << 0)
#define PORT_PMUX_PMUXO_MASK                     (0xf << 4)
#define PORT_PMUX_PMUXO(val)                     ((val & 0xf) << 4)
    uint8_t pincfg[32];
#define PORT_PINCFG_PMUXEN                       (1 << 0)
#define PORT_PINCFG_INEN                         (1 << 1)
#define PORT_PINCFG_PULLEN                       (1 << 2)
#define PORT_PINCFG_DRVSTR                       (1 << 6)
} __attribute__ ((packed)) port_t;

#define IOBUS_BASE                               0x60000000
#if defined(__AT91SAML21__)
#define PORT_BASE                                0x40002800
#endif /* __AT91SAML21__ */
#if defined(__AT91SAMD20__)
#define PORT_BASE                                0x41004400
#endif /* __AT91SAMD20__ */
#define PORT_OFFSET                              0x80

#define PORTA                                    ((volatile port_t *)(PORT_BASE + (PORT_OFFSET * 0)))
#define PORTB                                    ((volatile port_t *)(PORT_BASE + (PORT_OFFSET * 1)))
#define PORTC                                    ((volatile port_t *)(PORT_BASE + (PORT_OFFSET * 2)))
#define PORTD                                    ((volatile port_t *)(PORT_BASE + (PORT_OFFSET * 3)))

#define PORTA_IOBUS                              ((volatile port_t *)(IOBUS_BASE + (PORT_OFFSET * 0)))
#define PORTB_IOBUS                              ((volatile port_t *)(IOBUS_BASE + (PORT_OFFSET * 1)))
#define PORTC_IOBUS                              ((volatile port_t *)(IOBUS_BASE + (PORT_OFFSET * 2)))
#define PORTD_IOBUS                              ((volatile port_t *)(IOBUS_BASE + (PORT_OFFSET * 3)))

#define PORT_OUTPUT                              1
#define PORT_INPUT                               0
#define PORT_HIGH                                1
#define PORT_LOW                                 0
#define PORT_DRIVE_HIGH                          1
#define PORT_DRIVE_LOW                           0
#define PORT_PULLUP                              1
#define PORT_PULLDOWN                            0


void saml_port_clocks_init(void);

static inline void port_dir(volatile port_t *port, uint32_t pin, uint32_t isout)
{
    if (isout)
    {
        port->dirset = (1 << pin);
        port->pincfg[pin] |= PORT_PINCFG_INEN;
    }
    else
    {
        port->dirclr = (1 << pin);
        port->pincfg[pin] &= ~PORT_PINCFG_INEN;
    }
}

static inline void port_set(volatile port_t *port, uint32_t pin, uint32_t ishigh)
{
    if (ishigh)
    {
        port->outset = (1 << pin);
    }
    else
    {
        port->outclr = (1 << pin);
    }
}

static inline uint32_t port_get(volatile port_t *port, uint32_t pin)
{
    if (port->in & (1 << pin))
    {
        return 1;
    }

    return 0;
}

static inline void port_pull_disable(volatile port_t *port, uint32_t pin)
{
    port->pincfg[pin] &= ~PORT_PINCFG_PULLEN;
}

static inline void port_pull_enable(volatile port_t *port, uint32_t pin)
{
    // Pullups are only available if the pin is configured as a input.
    port_dir(port, pin, PORT_INPUT);
    port->pincfg[pin] |= PORT_PINCFG_PULLEN;
}

static inline void port_pull_direction(volatile port_t *port, uint32_t pin, uint32_t ishigh)
{
    // The pull direction is governed by the output state
    port_set(port, pin, ishigh);
}

static inline void port_strength(volatile port_t *port, uint32_t pin, uint32_t isstrong)
{
    if (isstrong)
    {
        port->pincfg[pin] |= PORT_PINCFG_DRVSTR;
    }
    else
    {
        port->pincfg[pin] &= ~PORT_PINCFG_DRVSTR;
    }
}

static inline void port_peripheral_enable(volatile port_t *port, uint32_t pin, uint8_t peripheral)
{
    uint8_t slot = pin >> 1;
    uint8_t odd = pin & 1;

    port_dir(port, pin, PORT_INPUT);

    if (!odd)
    {
        port->pmux[slot] &= ~PORT_PMUX_PMUXE_MASK;
        port->pmux[slot] |= PORT_PMUX_PMUXE(peripheral);
    }
    else
    {
        port->pmux[slot] &= ~PORT_PMUX_PMUXO_MASK;
        port->pmux[slot] |= PORT_PMUX_PMUXO(peripheral);
    }

    port->pincfg[pin] |= PORT_PINCFG_PMUXEN;
}

static inline void port_peripheral_disable(volatile port_t *port, uint32_t pin)
{
    port->pincfg[pin] &= ~PORT_PINCFG_PMUXEN;
}


typedef struct eic
{
    uint8_t  ctrla;
#define EIC_CTRLA_SWRST                          (1 << 0)
#define EIC_CTRLA_ENABLE                         (1 << 1)
#define EIC_CTRLA_CKSEL                          (1 << 4)
    uint8_t  nmictrl;
#define EIC_NMICTRL_NMISENSE(val)                (((val) & 0x7) << 0)
#define EIC_NMICTRL_NMIFILTEN                    (1 << 3)
#define EIC_NMICTRL_ASYNC                        (1 << 4)
    uint16_t nmiflag;
#define EIC_NMIFLAG_NMI                          (1 << 0)
    uint32_t syncbusy;
#define EIC_SYNCBUSY_SWRST                       (1 << 0)
#define EIC_SYNCBUSY_ENABLE                      (1 << 1)
    uint32_t evctrl;
#define EIC_EVCTRL_EXTINTEO(val)                 (1 << val)
    uint32_t intenclr;
#define EIC_INTENCLR_EXTINT(val)                 (1 << val)
    uint32_t intenset;
#define EIC_INTENSET_EXTINT(val)                 (1 << val)
    uint32_t intflag;
#define EIC_INTFLAG_EXTINT(val)                  (1 << val)
    uint32_t async;
#define EIC_ASYNC_ASYNCH(val)                    (1 << val)
    uint32_t confign0;
    uint32_t confign1;
#define EIC_CONFIG_SENSE_MASK(n)                 (0x7 << (n << 2))
#define EIC_CONFIG_SENSE(n, val)                 (((val) & 0x7) << (n << 2))
#define EIC_CONFIG_FILT(n)                       ((1 << 3) << (n << 2))
} __attribute__ ((packed)) eic_t;

#define EIC_BASE                                 0x40002400
#define EIC                                      ((volatile eic_t *)EIC_BASE)


typedef void (*eic_callback_t)(void *);
typedef struct
{
    eic_callback_t callback;
    void *arg;
} ext_int_t;

#define EIC_INTS_MAX                             16

#define EIC_EDGE_NONE                            0x0
#define EIC_EDGE_RISE                            0x1
#define EIC_EDGE_FALL                            0x2
#define EIC_EDGE_BOTH                            0x3
#define EIC_LEVEL_HIGH                           0x4
#define EIC_LEVEL_LOW                            0x5

static inline void eic_sense(uint8_t intnum, uint8_t type)
{
    volatile uint32_t *config = &EIC->confign0;
    uint8_t index = intnum;

    config = &EIC->confign0;
    if (intnum > 7)
    {
        config = &EIC->confign0;
        index = intnum - 8;
    }

    *config &= ~EIC_CONFIG_SENSE_MASK(index);
    *config |= EIC_CONFIG_SENSE(index, type);
}

static inline void eic_int_enable(uint8_t intnum)
{
    EIC->intenset = EIC_INTENSET_EXTINT(intnum);
}

static inline void eic_int_disable(uint8_t intnum)
{
    EIC->intenclr = EIC_INTENSET_EXTINT(intnum);
}

void eic_int_setup(uint8_t intnum, ext_int_t *ext_int, uint8_t sense_type);

// The enable must happen _after_ configuring the setup. Unfortunately,
// once enabled the sense configuration can no longer be changed.
void eic_enable(void);


#endif /* __SAML_PORT_H__ */
