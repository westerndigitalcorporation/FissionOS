#
# cortexm.py
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


# ARM Tools

import SCons
import os
import string
import struct
import array
import tarfile

def cortexa_flags(env):
    gnu_tools = ['gcc', 'g++', 'gnulink', 'ar', 'gas']
    for tool in gnu_tools:
        env.Tool(tool)   # This will ensure the normal Program, Object, etc.. work

    cflags = '''
        -D__EMBEDDED_STANDALONE__
        -O2
        -Wall
        -g
        -fno-omit-frame-pointer
        -fdiagnostics-color=always
    '''.split()

    env['CFLAGS'] = cflags
    env['CPPPATH' ] = [
        '#/common',
    ]

    env['CC'] = 'arm-none-eabi-gcc'
    env['LINK'] = 'arm-none-eabi-gcc'
    env['AR'] = 'arm-none-eabi-ar'
    env['RANLIB'] = 'arm-none-eabi-ranlib'
    env['OBJCOPY'] = 'arm-none-eabi-objcopy'

    # Build a specially formed command line to include the libs
    # given the quirks of gcc.  ${LINKSCRIPT} must be defined for
    # the appropriate environment (app, bootloader, etc.).
    link = '''
        -nostartfiles
        -T${LINKSCRIPT}
        '''.split()

    env.Append(
        LINKFLAGS = link
    )

    # Verbose?
    if not env['V']:
        env.Append(
            VERBOSE =      "/dev/null",

            CCCOMSTR     = "CC       ${TARGET}",
            ARCOMSTR     = "AR       ${TARGET}",
            RANLIBCOMSTR = "RANLIB   ${TARGET}",
            BINCOMSTR    = "Bin      ${TARGET}",
            HEXCOMSTR    = "Hex      ${TARGET}",
            ELFCOMSTR    = "Elf      ${TARGET}",
            IMGCOMSTR    = "Img      ${TARGET}",
            APPCOMSTR    = "App      ${TARGET}",
            UNTARCOMSTR  = "UnTar    ${SOURCE}",
            BUILDCOMSTR  = "Build    ${TARGETS}",
            APPLYCOMSTR  = "Apply    ${SOURCE}",
            USERSIGCOMSTR= "Usersig  ${SOURCE}",
            SECTIONCOMSTR= "Sections ${TARGET}",
        )
    else:
        env.Append(
            VERBOSE =      "/dev/stdout"
        )


#
# Builder python functions
#
def cortexa_builders(env):
    env.Append(BUILDERS = {
        'Elf': SCons.Builder.Builder(
            action = SCons.Action.Action("${LINK} -lc -lm ${LINKFLAGS} -Wl,--start-group ${SOURCES} -Wl,--end-group -o ${TARGET}", "${ELFCOMSTR}"),
            suffix = '.elf'
        ),
        'Hex' : SCons.Builder.Builder(
            action = SCons.Action.Action("${OBJCOPY} -O ihex ${SOURCES} ${TARGET}", "${HEXCOMSTR}"),
            suffix = '.hex'
        ),
        'Bin' : SCons.Builder.Builder(
            action = SCons.Action.Action("${OBJCOPY} -O binary ${SOURCES} ${TARGET}", "${BINCOMSTR}"),
            suffix = '.bin'
        ),
        'Usersig' : SCons.Builder.Builder(
            action = SCons.Action.Action(""),
        ),

    })


#
# The following are required functions when using this via tools= in Environment()
#
def exists(env):
    return true

def generate(env, **kwargs):
    [ f(env) for f in (cortexa_flags, cortexa_builders) ]


