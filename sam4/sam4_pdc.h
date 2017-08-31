/*
 * sam4_pdc.h
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


#ifndef __SAM4S_PDC_H__
#define __SAM4S_PDC_H__


/**
 *  Peripheral DMA Controller Register Definition
 *
 *  This register set is typically found at offset 0x100 in each of the
 *  peripheral register spaces that support DMA.
 */
typedef struct pdc
{
    void     *rpr;                                           /**< Receive Pointer             */
    uint32_t  rcr;                                           /**< Receive Counter             */
    void     *tpr;                                           /**< Transmit Pointer            */
    uint32_t  tcr;                                           /**< Transmit Counter            */
    void     *rnpr;                                          /**< Recieve Next Pointer        */
    uint32_t  rncr;                                          /**< Receive Next Counter        */
    void     *tnpr;                                          /**< Transmit Next Pointer       */
    uint32_t  tncr;                                          /**< Transmit Next Counter       */
    uint32_t  ptcr;                                          /**< Transfer Control            */
#define PDC_PTCR_RXTEN                           (1 << 0)    /**< Receive Transfer Enable     */
#define PDC_PTCR_RXDIS                           (1 << 1)    /**< Receive Transfer Disable    */
#define PDC_PTCR_TXTEN                           (1 << 8)    /**< Transmit Transfer Enable    */
#define PDC_PTCR_TXDIS                           (1 << 9)    /**< Transmit Transfer Disable   */
    uint32_t  ptsr;                                          /**< Transfer Status             */
#define PDC_PTSR_RXTEN                           (1 << 0)    /**< Receive Transfers Enabled   */
#define PDC_PTSR_TXTEN                           (1 << 8)    /**< Transmit Transfers Enabled  */
} __attribute__ ((packed)) pdc_t;


#endif /* __SAM_PDC_H__ */

