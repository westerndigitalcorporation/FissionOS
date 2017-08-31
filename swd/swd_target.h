/*
 * swd_target.h
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


#ifndef __SWD_TARGET_H__
#define __SWD_TARGET_H__

#include "swd.h"

#include "swd_chipid.h"

int swd_target_halt(swd_t *instance);
int swd_target_is_halted(swd_t *instance);
int swd_target_resume(swd_t *instance);
int swd_target_reset(swd_t *instance);
int swd_target_step(swd_t *instance);
int swd_target_regs(swd_t *instance, uint32_t *regbuf);
int swd_target_set_reg(swd_t *instance, uint8_t reg, uint32_t value);
int swd_target_breakpoint_set(swd_t *instance, int bpnum, uint32_t addr);
int swd_target_breakpoint_clear(swd_t *instance, int bpnum);

int swd_target_read_uint8(swd_t *instance, uint32_t addr, uint8_t *data);
int swd_target_write_uint8(swd_t *instance, uint32_t addr, uint8_t data);

#endif /* __SWD_TARGET_H__ */

