/*
 * swd_bitbang.h
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


#ifndef __SWD_BITBANG_H__
#define __SWD_BITBANG_H__


int swd_bitbang_init(swd_t *instance);

int swd_bitbang_start(swd_t *instance);
int swd_bitbang_stop(swd_t *instance);
void swd_bitbang_flush(swd_t *instance);

int swd_bitbang_request_recv(swd_t *instance, uint8_t request, uint8_t *ack, uint32_t *data);
int swd_bitbang_request_xmit(swd_t *instance, uint8_t request, uint8_t *ack, uint32_t data);


/* The following functions must be implemented by the platform specific bitbang driver */
void swd_bitbang_clk_dir(struct swd *instance, int output);
void swd_bitbang_clk_set(struct swd *instance, int high);
void swd_bitbang_dio_dir(struct swd *instance, int output);
void swd_bitbang_dio_set(struct swd *instance, int high);
int swd_bitbang_dio_get(struct swd *instance);
void swd_bitbang_udelay(uint32_t usecs);


#endif /* __SWD_BITBANG_H__ */
