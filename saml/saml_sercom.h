/*
 * saml_sercom.h
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


#ifndef __SAML_SERCOM_H__
#define __SAML_SERCOM_H__

#ifdef __AT91SAML21__
typedef volatile struct sercom_usart
{
    uint32_t ctrla;
#define SERCOM_USART_CTRLA_SWRST                 (1 << 0)
#define SERCOM_USART_CTRLA_ENABLE                (1 << 1)
#define SERCOM_USART_CTRLA_MODE_EXTCLK           (0 << 2)
#define SERCOM_USART_CTRLA_MODE_INTCLK           (1 << 2)
#define SERCOM_USART_CTRLA_RUNSTDBY              (1 << 7)
#define SERCOM_USART_CTRLA_IBON                  (1 << 8)
#define SERCOM_USART_CTRLA_SAMPR_16X_ARITH       (0x0 << 13)
#define SERCOM_USART_CTRLA_SAMPR_16X_FRACT       (0x1 << 13)
#define SERCOM_USART_CTRLA_SAMPR_8X_ARITH        (0x2 << 13)
#define SERCOM_USART_CTRLA_SAMPR_8X_FRACT        (0x3 << 13)
#define SERCOM_USART_CTRLA_SAMPR_3X_ARITH        (0x4 << 13)
#define SERCOM_USART_CTRLA_TXPO_PAD0             (0x0 << 16)
#define SERCOM_USART_CTRLA_TXPO_PAD2             (0x1 << 16)
#define SERCOM_USART_CTRLA_TXPO_PAD0_RTSCTS      (0x2 << 16)
#define SERCOM_USART_CTRLA_RXPO_PAD0             (0x0 << 20)
#define SERCOM_USART_CTRLA_RXPO_PAD1             (0x1 << 20)
#define SERCOM_USART_CTRLA_RXPO_PAD2             (0x2 << 20)
#define SERCOM_USART_CTRLA_RXPO_PAD3             (0x3 << 20)
#define SERCOM_USART_CTRLA_SAMPA(val)            ((val & 0x3) << 22)
#define SERCOM_USART_CTRLA_FORM_FRAME            (0x0 << 24)
#define SERCOM_USART_CTRLA_FORM_FRAME_PARITY     (0x1 << 24)
#define SERCOM_USART_CTRLA_FORM_AUTO_BAUD        (0x4 << 24)
#define SERCOM_USART_CTRLA_FORM_AUTO_BAUD_PARITY (0x5 << 24)
#define SERCOM_USART_CTRLA_CMODE_ASYNC           (0 << 28)
#define SERCOM_USART_CTRLA_CMODE_SYNC            (1 << 28)
#define SERCOM_USART_CTRLA_CPOL                  (1 << 29)
#define SERCOM_USART_CTRLA_DORD_MSB              (0 << 30)
#define SERCOM_USART_CTRLA_DORD_LSB              (1 << 30)
    uint32_t ctrlb;
#define SERCOM_USART_CTRLB_CHSIZE_8BITS          (0x0 << 0)
#define SERCOM_USART_CTRLB_CHSIZE_9BITS          (0x1 << 0)
#define SERCOM_USART_CTRLB_CHSIZE_5BITS          (0x5 << 0)
#define SERCOM_USART_CTRLB_CHSIZE_6BITS          (0x6 << 0)
#define SERCOM_USART_CTRLB_CHSIZE_7BITS          (0x7 << 0)
#define SERCOM_USART_CTRLB_SBMODE_2BITS          (1 << 6)
#define SERCOM_USART_CTRLB_SBMODE_1BIT           (0 << 6)
#define SERCOM_USART_CTRLB_COLDEN                (1 << 8)
#define SERCOM_USART_CTRLB_SFDE                  (1 << 9)
#define SERCOM_USART_CTRLB_ENC                   (1 << 10)
#define SERCOM_USART_CTRLB_PMODE_EVEN            (0 << 13)
#define SERCOM_USART_CTRLB_PMODE_ODD             (1 << 13)
#define SERCOM_USART_CTRLB_TXEN                  (1 << 16)
#define SERCOM_USART_CTRLB_RXEN                  (1 << 17)
    uint8_t  resvd_0x08[4];
    uint16_t baud;
    uint8_t  rxpl;
    uint8_t  resvd_0x0f[5];
    uint8_t  intenclr;
#define SERCOM_USART_INTENCLR_DRE                (1 << 0)
#define SERCOM_USART_INTENCLR_TXC                (1 << 1)
#define SERCOM_USART_INTENCLR_RXC                (1 << 2)
#define SERCOM_USART_INTENCLR_RXS                (1 << 3)
#define SERCOM_USART_INTENCLR_CTSIC              (1 << 4)
#define SERCOM_USART_INTENCLR_RXBRK              (1 << 5)
#define SERCOM_USART_INTENCLR_ERROR              (1 << 7)
    uint8_t  resvd_0x15;
    uint8_t  intenset;
#define SERCOM_USART_INTENSET_DRE                (1 << 0)
#define SERCOM_USART_INTENSET_TXC                (1 << 1)
#define SERCOM_USART_INTENSET_RXC                (1 << 2)
#define SERCOM_USART_INTENSET_RXS                (1 << 3)
#define SERCOM_USART_INTENSET_CTSIC              (1 << 4)
#define SERCOM_USART_INTENSET_RXBRK              (1 << 5)
#define SERCOM_USART_INTENSET_ERROR              (1 << 7)
    uint8_t  resvd_0x17;
    uint8_t  intflag;
#define SERCOM_USART_INTFLAG_DRE                 (1 << 0)
#define SERCOM_USART_INTFLAG_TXC                 (1 << 1)
#define SERCOM_USART_INTFLAG_RXC                 (1 << 2)
#define SERCOM_USART_INTFLAG_RXS                 (1 << 3)
#define SERCOM_USART_INTFLAG_CTSIC               (1 << 4)
#define SERCOM_USART_INTFLAG_RXBRK               (1 << 5)
#define SERCOM_USART_INTFLAG_ERROR               (1 << 7)
    uint8_t  resvd_0x19;
    uint16_t status;
#define SERCOM_USART_STATUS_PERR                 (1 << 0)
#define SERCOM_USART_STATUS_FERR                 (1 << 1)
#define SERCOM_USART_STATUS_BUFOVF               (1 << 2)
#define SERCOM_USART_STATUS_CTS                  (1 << 3)
#define SERCOM_USART_STATUS_ISF                  (1 << 4)
#define SERCOM_USART_STATUS_COLL                 (1 << 5)
    uint32_t syncbusy;
#define SERCOM_USART_SYNCBUSY_SWRST              (1 << 0)
#define SERCOM_USART_SYNCBUSY_ENABLE             (1 << 1)
#define SERCOM_USART_SYNCBUSY_CTRLB              (1 << 2)
    uint8_t  resvd_0x20[8];
    uint16_t data;
    uint8_t  resvd_0x2a[6];
    uint8_t  dbgctrl;
#define SERCOM_USART_DBGCTRL_DBGSTOP             (1 << 0)
} __attribute__ ((packed)) sercom_usart_t;
#endif /* __AT91SAML21__ */

#ifdef __AT91SAMD20__
typedef volatile struct sercom_usart
{
    uint32_t ctrla;
#define SERCOM_USART_CTRLA_SWRST                 (1 << 0)
#define SERCOM_USART_CTRLA_ENABLE                (1 << 1)
#define SERCOM_USART_CTRLA_MODE_EXTCLK           (0 << 2)
#define SERCOM_USART_CTRLA_MODE_INTCLK           (1 << 2)
#define SERCOM_USART_CTRLA_RUNSTDBY              (1 << 7)
#define SERCOM_USART_CTRLA_IBON                  (1 << 8)
#define SERCOM_USART_CTRLA_SAMPR_16X_ARITH       (0x0 << 13)
#define SERCOM_USART_CTRLA_SAMPR_16X_FRACT       (0x1 << 13)
#define SERCOM_USART_CTRLA_SAMPR_8X_ARITH        (0x2 << 13)
#define SERCOM_USART_CTRLA_SAMPR_8X_FRACT        (0x3 << 13)
#define SERCOM_USART_CTRLA_SAMPR_3X_ARITH        (0x4 << 13)
#define SERCOM_USART_CTRLA_TXPO_PAD0             (0x0 << 16)
#define SERCOM_USART_CTRLA_TXPO_PAD2             (0x1 << 16)
#define SERCOM_USART_CTRLA_TXPO_PAD0_RTSCTS      (0x2 << 16)
#define SERCOM_USART_CTRLA_RXPO_PAD0             (0x0 << 20)
#define SERCOM_USART_CTRLA_RXPO_PAD1             (0x1 << 20)
#define SERCOM_USART_CTRLA_RXPO_PAD2             (0x2 << 20)
#define SERCOM_USART_CTRLA_RXPO_PAD3             (0x3 << 20)
#define SERCOM_USART_CTRLA_SAMPA(val)            ((val & 0x3) << 22)
#define SERCOM_USART_CTRLA_FORM_FRAME            (0x0 << 24)
#define SERCOM_USART_CTRLA_FORM_FRAME_PARITY     (0x1 << 24)
#define SERCOM_USART_CTRLA_FORM_AUTO_BAUD        (0x4 << 24)
#define SERCOM_USART_CTRLA_FORM_AUTO_BAUD_PARITY (0x5 << 24)
#define SERCOM_USART_CTRLA_CMODE_ASYNC           (0 << 28)
#define SERCOM_USART_CTRLA_CMODE_SYNC            (1 << 28)
#define SERCOM_USART_CTRLA_CPOL                  (1 << 29)
#define SERCOM_USART_CTRLA_DORD_MSB              (0 << 30)
#define SERCOM_USART_CTRLA_DORD_LSB              (1 << 30)
    uint32_t ctrlb;
#define SERCOM_USART_CTRLB_CHSIZE_8BITS          (0x0 << 0)
#define SERCOM_USART_CTRLB_CHSIZE_9BITS          (0x1 << 0)
#define SERCOM_USART_CTRLB_CHSIZE_5BITS          (0x5 << 0)
#define SERCOM_USART_CTRLB_CHSIZE_6BITS          (0x6 << 0)
#define SERCOM_USART_CTRLB_CHSIZE_7BITS          (0x7 << 0)
#define SERCOM_USART_CTRLB_SBMODE_2BITS          (1 << 6)
#define SERCOM_USART_CTRLB_SBMODE_1BIT           (0 << 6)
#define SERCOM_USART_CTRLB_COLDEN                (1 << 8)
#define SERCOM_USART_CTRLB_SFDE                  (1 << 9)
#define SERCOM_USART_CTRLB_ENC                   (1 << 10)
#define SERCOM_USART_CTRLB_PMODE_EVEN            (0 << 13)
#define SERCOM_USART_CTRLB_PMODE_ODD             (1 << 13)
#define SERCOM_USART_CTRLB_TXEN                  (1 << 16)
#define SERCOM_USART_CTRLB_RXEN                  (1 << 17)
    uint8_t  dbgctrl;
#define SERCOM_USART_DBGCTRL_DBGSTOP             (1 << 0)
    uint8_t  resvd_0x09;
    uint16_t baud;
    uint8_t  intenclr;
#define SERCOM_USART_INTENCLR_DRE                (1 << 0)
#define SERCOM_USART_INTENCLR_TXC                (1 << 1)
#define SERCOM_USART_INTENCLR_RXC                (1 << 2)
#define SERCOM_USART_INTENCLR_RXS                (1 << 3)
    uint8_t  intenset;
#define SERCOM_USART_INTENSET_DRE                (1 << 0)
#define SERCOM_USART_INTENSET_TXC                (1 << 1)
#define SERCOM_USART_INTENSET_RXC                (1 << 2)
#define SERCOM_USART_INTENSET_RXS                (1 << 3)
    uint8_t  intflag;
#define SERCOM_USART_INTFLAG_DRE                 (1 << 0)
#define SERCOM_USART_INTFLAG_TXC                 (1 << 1)
#define SERCOM_USART_INTFLAG_RXC                 (1 << 2)
#define SERCOM_USART_INTFLAG_RXS                 (1 << 3)
    uint8_t  resvd_0x19;
    uint16_t status;
#define SERCOM_USART_STATUS_PERR                 (1 << 0)
#define SERCOM_USART_STATUS_FERR                 (1 << 1)
#define SERCOM_USART_STATUS_BUFOVF               (1 << 2)
#define SERCOM_USART_STATUS_SYNCBUSY             (1 << 2)
    uint8_t  resvd_0x12[6];
    uint16_t data;
} __attribute__ ((packed)) sercom_usart_t;
#endif /* __AT91SAMD20__ */

#define UART_RX_BUFFER_SIZE                      8

struct uart_drv;
typedef void (*uart_callback_t)(struct uart_drv *uart, void *arg);
typedef struct uart_drv
{
    volatile sercom_usart_t *dev;
    uart_callback_t tx_cb;
    void *tx_arg;
    uart_callback_t rx_cb;
    void *rx_arg;
    uint8_t rxbuf[UART_RX_BUFFER_SIZE];
    volatile uint8_t rxbuf_start;
    volatile uint8_t rxbuf_end;
} uart_drv_t;


void sercom_usart_async_init(uart_drv_t *uart, uint8_t peripheral_id,
                             uint32_t clockrate, uint32_t baudrate,
                             uint32_t chsize, uint32_t sbmode, uint32_t form, uint32_t pmode,
                             uint32_t txpo, uint32_t rxpo);
void sercom_usart_disable(volatile sercom_usart_t *usart);


void uart_init(uart_drv_t *uart);
void uart_send_wait(uart_drv_t *uart, char *data, uint32_t len);
int uart_recv(uart_drv_t *uart, uint8_t *data, int maxlen);


typedef volatile struct sercom_i2c
{
    uint32_t ctrla;
#define SERCOM_I2C_CTRLA_SWRST                   (1 << 0)
#define SERCOM_I2C_CTRLA_ENABLE                  (1 << 1)
#define SERCOM_I2C_CTRLA_SLAVE                   (0x4 << 2)
#define SERCOM_I2C_CTRLA_MASTER                  (0x5 << 2)
#define SERCOM_I2C_CTRLA_RUNSTDBY                (1 << 7)
#define SERCOM_I2C_CTRLA_PINOUT                  (1 << 16)
#define SERCOM_I2C_CTRLA_SDAHOLD_DIS             (0x0 << 20)
#define SERCOM_I2C_CTRLA_SDAHOLD_75              (0x1 << 20)
#define SERCOM_I2C_CTRLA_SDAHOLD_450             (0x2 << 20)
#define SERCOM_I2C_CTRLA_SDAHOLD_600             (0x3 << 20)
#define SERCOM_I2C_CTRLA_SEXTTOEN                (1 << 23)
#define SERCOM_I2C_CTRLA_SPEED_SM                (0x0 << 24)
#define SERCOM_I2C_CTRLA_SPEED_FM                (0x1 << 24)
#define SERCOM_I2C_CTRLA_SPEED_HSM               (0x2 << 24)
#define SERCOM_I2C_CTRLA_SCLSM                   (1 << 27)
#define SERCOM_I2C_CTRLA_INACTOUT_DIS            (0x0 << 28)
#define SERCOM_I2C_CTRLA_INACTOUT_55US           (0x1 << 28)
#define SERCOM_I2C_CTRLA_INACTOUT_105US          (0x2 << 28)
#define SERCOM_I2C_CTRLA_INACTOUT_205US          (0x3 << 28)
#define SERCOM_I2C_CTRLA_LOWTOUT                 (1 << 30)
    uint32_t ctrlb;
#define SERCOM_I2C_CTRLB_SMEN                    (1 << 8)
#define SERCOM_I2C_CTRLB_GCMD                    (1 << 9)
#define SERCOM_I2C_CTRLB_AACKEN                  (1 << 10)
#define SERCOM_I2C_CTRLB_AMODE_COMP              (0x2 << 16)
#define SERCOM_I2C_CTRLB_AMODE_RESP              (0x3 << 16)
#define SERCOM_I2C_CTRLB_CMD_MASK                (0x3 << 16)         // master
#define SERCOM_I2C_CTRLB_CMD_ACK_RESTART         (0x1 << 16)         // master
#define SERCOM_I2C_CTRLB_CMD_ACK                 (0x2 << 16)         // master
#define SERCOM_I2C_CTRLB_CMD_DRDY                (0x2 << 16)         // slave
#define SERCOM_I2C_CTRLB_CMD_STOP                (0x3 << 16)         // master
#define SERCOM_I2C_CTRLB_AMODE_MASK              (0x0 << 14)
#define SERCOM_I2C_CTRLB_AMODE_2_ADDRS           (0x1 << 14)
#define SERCOM_I2C_CTRLB_AMODE_RANGE             (0x2 << 14)
#define SERCOM_I2C_CTRLB_ACKACT                  (1 << 18)
    uint8_t  resvd_0x08[4];
    uint32_t baud;
#define SERCOM_I2C_BAUD_BAUD(val)                (((val) & 0xff) << 0)
#define SERCOM_I2C_BAUD_BAUDLOW(val)             (((val) & 0xff) << 8)
#define SERCOM_I2C_BAUD_HSBAUD(val)              (((val) & 0xff) << 16)
#define SERCOM_I2C_BAUD_HSBAUDLOW(val)           (((val) & 0xff) << 24)
    uint8_t  resvd_0x10[4];
    uint8_t  intenclr;
#define SERCOM_I2C_INTENCLR_PREC                 (1 << 0)
#define SERCOM_I2C_INTENCLR_MB                   (1 << 0)
#define SERCOM_I2C_INTENCLR_AMATCH               (1 << 1)
#define SERCOM_I2C_INTENCLR_SB                   (1 << 1)
#define SERCOM_I2C_INTENCLR_DRDY                 (1 << 2)
#define SERCOM_I2C_INTENCLR_ERROR                (1 << 7)
    uint8_t  resvd_0x15;
    uint8_t  intenset;
#define SERCOM_I2C_INTENSET_PREC                 (1 << 0)
#define SERCOM_I2C_INTENSET_MB                   (1 << 0)
#define SERCOM_I2C_INTENSET_AMATCH               (1 << 1)
#define SERCOM_I2C_INTENSET_SB                   (1 << 1)
#define SERCOM_I2C_INTENSET_DRDY                 (1 << 2)
#define SERCOM_I2C_INTENSET_ERROR                (1 << 7)
    uint8_t  resvd_0x17;
    uint8_t  intflag;
#define SERCOM_I2C_INTFLAG_PREC                  (1 << 0)
#define SERCOM_I2C_INTFLAG_MB                    (1 << 0)
#define SERCOM_I2C_INTFLAG_AMATCH                (1 << 1)
#define SERCOM_I2C_INTFLAG_SB                    (1 << 1)
#define SERCOM_I2C_INTFLAG_DRDY                  (1 << 2)
#define SERCOM_I2C_INTFLAG_ERROR                 (1 << 7)
    uint8_t  resvd_0x19;
    uint16_t status;
#define SERCOM_I2C_STATUS_BUSERR                 (1 << 0)
#define SERCOM_I2C_STATUS_COLL                   (1 << 1)
#define SERCOM_I2C_STATUS_ARBLOST                (1 << 1)             // master
#define SERCOM_I2C_STATUS_RXNACK                 (1 << 2)
#define SERCOM_I2C_STATUS_DIR                    (1 << 3)             // slave
#define SERCOM_I2C_STATUS_SR                     (1 << 4)             // slave
#define SERCOM_I2C_STATUS_BUSSTATE(reg)          ((reg >> 4) & 0x3)   // master
#define SERCOM_I2C_STATUS_BUSSTATE_IDLE          (0x1 << 4)           // master
#define SERCOM_I2C_STATUS_LOWTOUT                (1 << 6)
#define SERCOM_I2C_STATUS_CLKHOLD                (1 << 7)
#define SERCOM_I2C_STATUS_MEXTTOUT               (1 << 8)             // master
#define SERCOM_I2C_STATUS_SEXTTOUT               (1 << 9)
#define SERCOM_I2C_STATUS_LENERR                 (1 << 10)
    uint32_t syncbusy;
#define SERCOM_I2C_SYNCBUSY_SWRST                (1 << 0)
#define SERCOM_I2C_SYNCBUSY_ENABLE               (1 << 1)
#define SERCOM_I2C_SYNCBUSY_SYSOP                (1 << 1)             // master
    uint8_t  resvd_0x20[4];
    uint32_t addr;
#define SERCOM_I2C_ADDR_GENCEN                   (1 << 0)
#define SERCOM_I2C_ADDR_ADDR(val)                ((val & 0x3ff) << 1)
#define SERCOM_I2C_ADDR_LENEN                    (1 << 13)            // master
#define SERCOM_I2C_ADDR_HS                       (1 << 14)
#define SERCOM_I2C_ADDR_TENBITEN                 (1 << 15)
#define SERCOM_I2C_ADDR_ADDRMASK(val)            ((val & 0x3ff) << 17)
#define SERCOM_I2C_ADDR_LEN(val)                 ((val & 0xff) << 16) // master
#define SERCOM_I2C_ADDR_MASTER_ADDR(val)         (((val) & 0x3ff) << 0) // master
    uint16_t data;
    uint8_t resvd_0x28[6];
    uint8_t dbgctrl;
} __attribute__ ((packed)) sercom_i2c_t;

#if defined(__AT91SAML21__)
#define SERCOM0_ADDR                             0x42000000
#define SERCOM1_ADDR                             0x42000400
#define SERCOM2_ADDR                             0x42000800
#define SERCOM3_ADDR                             0x42000c00
#define SERCOM4_ADDR                             0x42001000
#define SERCOM5_ADDR                             0x43000400
#endif /* __AT91SAML21__ */
#if defined(__AT91SAMD20__)
#define SERCOM0_ADDR                             0x42000800
#define SERCOM1_ADDR                             0x42000c00
#define SERCOM2_ADDR                             0x42001000
#define SERCOM3_ADDR                             0x42001400
#define SERCOM4_ADDR                             0x42001800
#define SERCOM5_ADDR                             0x42001c00
#endif /* __AT91SAMD20__ */

#define SERCOM0_USART                            ((volatile sercom_usart_t *)SERCOM0_ADDR)
#define SERCOM1_USART                            ((volatile sercom_usart_t *)SERCOM1_ADDR)
#define SERCOM2_USART                            ((volatile sercom_usart_t *)SERCOM2_ADDR)
#define SERCOM3_USART                            ((volatile sercom_usart_t *)SERCOM3_ADDR)
#define SERCOM4_USART                            ((volatile sercom_usart_t *)SERCOM4_ADDR)
#define SERCOM5_USART                            ((volatile sercom_usart_t *)SERCOM5_ADDR)

#define SERCOM0_I2C                              ((volatile sercom_i2c_t *)SERCOM0_ADDR)
#define SERCOM1_I2C                              ((volatile sercom_i2c_t *)SERCOM1_ADDR)
#define SERCOM2_I2C                              ((volatile sercom_i2c_t *)SERCOM2_ADDR)
#define SERCOM3_I2C                              ((volatile sercom_i2c_t *)SERCOM3_ADDR)
#define SERCOM4_I2C                              ((volatile sercom_i2c_t *)SERCOM4_ADDR)
#define SERCOM5_I2C                              ((volatile sercom_i2c_t *)SERCOM5_ADDR)

#define SERCOM_COUNT                             6


/*
 * All callbacks occur in interrupt context, applications should defer processing
 * to workqueues or tasks.
 */
struct twi_drv;
typedef void (*twi_master_callback_t)(struct twi_drv *twi, void *arg, int result);
typedef void (*twi_slave_rx_t)(struct twi_drv *twi, void *arg, uint8_t data);
typedef void (*twi_slave_tx_t)(struct twi_drv *twi, void *arg);
typedef void (*twi_slave_start_t)(struct twi_drv *twi, void *arg);
typedef void (*twi_slave_done_t)(struct twi_drv *twi, void *arg);
typedef void (*twi_slave_nak_t)(struct twi_drv *twi, void *arg);

typedef struct twi_drv
{
    /* The following need to be iniialized by the user */
    volatile sercom_i2c_t *i2c;

    volatile uint8_t type;
#define TWI_DRV_TYPE_MASTER                       0
#define TWI_DRV_TYPE_SLAVE                        1

    volatile uint32_t state;
#define TWI_DRV_STATE_IDLE                        0
#define TWI_DRV_STATE_WRITE_DATA                  2
#define TWI_DRV_STATE_STOP                        3
#define TWI_DRV_STATE_READ_DATA                   5
#define TWI_DRV_STATE_SLAVE_BUSY                  6

    uint8_t slave_addr;
    uint8_t dst_addr;

    /* Master */
    twi_master_callback_t complete;
    void *master_arg;
    uint32_t read_len;
    uint8_t *read_buffer;
    uint32_t write_len;
    uint8_t *write_buffer;

    /* Slave */
    twi_slave_start_t start;
    twi_slave_tx_t read;
    twi_slave_rx_t write;
    twi_slave_done_t done;
    twi_slave_nak_t nak;
    void *slave_arg;

    uint32_t clockrate;
    uint32_t baudrate;
} twi_drv_t;

int twi_init(twi_drv_t *twi, uint8_t peripheral_id,
             uint32_t clockrate, uint32_t baudrate);
int twi_master_xfer(twi_drv_t *twi, uint8_t addr,
                    uint8_t *write_buf, uint32_t write_len,
                    uint8_t *read_buf, uint32_t read_len,
                    twi_master_callback_t complete, void *arg);
void twi_master_wait(twi_drv_t *twi);

void twi_slave_send(twi_drv_t *twi, uint8_t data);

#define CONSOLE_CMD_I2C                          \
    {                                            \
        .cmdstr = "i2c",                         \
        .callback = cmd_twi,                     \
        .usage = "  i2c <list | <devicenum> <addr> <read len> [write data ...]>\r\n",       \
        .help =                                  \
            "  Read and write data on a given I2c interface.\r\n"  \
            "    list       : Show available i2c devices.\r\n" \
            "    device     : I2c interface number.\r\n" \
            "    addr       : I2c slave address.\r\n" \
            "    read len   : Length in bytes to read from slave.\r\n" \
            "    write data : Bytes to write separated by spaces.\r\n", \
    }

int cmd_twi(uart_drv_t *uart, int argc, char *argv[]);


#endif /* __SAML_SERCOM_H__ */
