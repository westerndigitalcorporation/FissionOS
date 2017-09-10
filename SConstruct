#
# SConstruct
#
# 
# Copyright (c) 2013-2017 Western Digital Corporation or its affiliates.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. The name of the copyright holder nor the names of its contributors may not
#    be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jeremy Garff <jeremy.garff@sandisk.com>
#


import os

opts = Variables()
opts.Add(BoolVariable('V',
                      'Show more command information',
                      False))

cortexm_env = Environment(
    options = opts,
    tools = ['cortexm', 'version'],
    toolpath = ['.'],
    EXPORTDIR = '#export',
    ENV = { 'PATH' : os.environ['PATH'], 'TERM' : os.environ['TERM'] }
)
Help(opts.GenerateHelpText(cortexm_env))

linux_env = Environment(
    options = opts,
    tools = ['linux', 'version'],
    toolpath = ['.'],
    ENV = { 'PATH' : os.environ['PATH'], 'TERM' : os.environ['TERM'] }
)

cortexa_env = Environment(
    options = opts,
    tools = ['cortexa', 'version'],
    toolpath = ['.'],
    EXPORTDIR = '#export',
    ENV = { 'PATH' : os.environ['PATH'] }
)

cortexa53_env = Environment(
    options = opts,
    tools = ['cortexa53', 'version'],
    toolpath = ['.'],
    EXPORTDIR = '#export',
    ENV = { 'PATH' : os.environ['PATH'] }
)

tools_env = linux_env.Clone()

cortexm4_env = cortexm_env.Clone()
cortexm4_env.MergeFlags({
    'CFLAGS' : [ '-mcpu=cortex-m4' ],
    'LINKFLAGS' : [ '-mcpu=cortex-m4' ],
})

cortexm0_env = cortexm_env.Clone()
cortexm0_env.MergeFlags({
    'CFLAGS' : [ '-mcpu=cortex-m0' ],
    'LINKFLAGS' : [ '-mcpu=cortex-m0' ],
})

cortexa8_env = cortexa_env.Clone()
cortexa8_env.MergeFlags({
    'CFLAGS' : [ '-mcpu=cortex-a8' ],
    'LINKFLAGS' : [ '-mcpu=cortex-a8' ],
})

at91sam4sd_env = cortexm4_env.Clone()
at91sam4sd_env.MergeFlags({
    'CFLAGS' : [
        '-D__AT91SAM4S__',
        '-D__DUAL_BANK_FLASH__',
        '-D__CPU_DATA_CACHE__'
    ],
    'OBJPREFIX' : [ 'at91sam4sd_' ],
})

at91sam4s_env = cortexm4_env.Clone()
at91sam4s_env.MergeFlags({
    'CFLAGS' : [
        '-D__AT91SAM4S__',
    ],
    'OBJPREFIX' : [ 'at91sam4s_' ],
})

at91sam4e_env = cortexm4_env.Clone()
at91sam4e_env.MergeFlags({
    'CFLAGS' : [
        '-D__AT91SAM4E__',
        '-D__CPU_DATA_CACHE__'
    ],
    'OBJPREFIX' : [ 'at91sam4e_' ],
})

at91saml21_env = cortexm0_env.Clone()
at91saml21_env.MergeFlags({
    'CFLAGS' : [
        '-D__AT91SAML21__',
    ],
    'OBJPREFIX' : [ 'at91saml21_' ],
})

at91samd20_env = cortexm0_env.Clone()
at91samd20_env.MergeFlags({
    'CFLAGS' : [
        '-D__AT91SAMD20__',
    ],
    'OBJPREFIX' : [ 'at91samd20_' ],
})

atsamd53_env = cortexm0_env.Clone()
atsamd53_env.MergeFlags({
    'CFLAGS' : [
        '-D__ATSAMD53__',
    ],
    'OBJPREFIX' : [ 'atsamd53_' ],
})

imx6_env = cortexa8_env.Clone()
imx6_env.MergeFlags({
    'CFLAGS' : [ '-D__IMX6__' ],
    'OBJPREFIX' : [ 'imx6_' ],
})

zynqmp_env = cortexa53_env.Clone()
zynqmp_env.MergeFlags({
    'CFLAGS' : [ '-D__ZYNQ__' ],
    'OBJPREFIX' : [ 'zynqmp_' ],
})

targets = {
    'tools' : {
        'ENV' : tools_env,
        'LIBS' : [],
    },
    'at91sam4sd' : {
        'ENV' : at91sam4sd_env,
        'LIBS' : [],
    },
    'at91sam4s' : {
        'ENV' : at91sam4s_env,
        'LIBS' : [],
    },
    'at91sam4e' : {
        'ENV' : at91sam4e_env,
        'LIBS' : [],
    },
    'at91saml21' : {
        'ENV' : at91saml21_env,
        'LIBS' : [],
    },
    'at91samd20' : {
        'ENV' : at91samd20_env,
        'LIBS' : [],
    },
    'atsamd53' : {
        'ENV' : atsamd53_env,
        'LIBS' : [],
    },
    'imx6' : {
        'ENV' : imx6_env,
        'LIBS' : [],
    },
    'zynqmp' : {
        'ENV' : zynqmp_env,
        'LIBS' : [],
    }
}

Export('targets')
SConscript('SConscript')

