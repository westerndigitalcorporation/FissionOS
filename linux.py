#
# linux.py
#
# 
# Copyright (c) 2017 Western Digital Corporation or its affiliates.
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

def linux_flags(env):
    gnu_tools = ['gcc', 'g++', 'gnulink', 'ar', 'gas']
    for tool in gnu_tools:
        env.Tool(tool)   # This will ensure the normal Program, Object, etc.. work

    cflags = '''
        -O0
        -Werror
        -Wall
        -g
    '''.split()

    env['CFLAGS'] = cflags
    env['CPPPATH' ] = [
        '#/common',
    ]

    env['CC'] = 'gcc'
    env['LINK'] = 'gcc'
    env['AR'] = 'ar'
    env['RANLIB'] = 'ranlib'
    env['OBJCOPY'] = 'objcopy'

    # Verbose?
    if not env['V']:
        env.Append(
            VERBOSE =      "/dev/null",

            CCCOMSTR      = "CC       ${TARGET}",
            ARCOMSTR      = "AR       ${TARGET}",
            RANLIBCOMSTR  = "RANLIB   ${TARGET}",
            PROGRAMCOMSTR = "ELF      ${TARGET}",
        )
    else:
        env.Append(
            VERBOSE =      "/dev/stdout"
        )


#
# Builder python functions
#
def linux_builders(env):
    env.Append(BUILDERS = {
    })


#
# The following are required functions when using this via tools= in Environment()
#
def exists(env):
    return true

def generate(env, **kwargs):
    [ f(env) for f in (linux_flags, linux_builders) ]


