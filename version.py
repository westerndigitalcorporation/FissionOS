#
# version.py
#
# 
# Copyright (c) 2009-2017 Western Digital Corporation or its affiliates.
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


# Version Tool

import os
import string

import SCons

def exists(env):
    return 1

def add_builders(env):

    def read_version(path):
        f = open(path, 'r')
        try:
            major, minor, micro = f.readline().strip().split('.')
        except:
            major, minor, micro = 0, 0, 0
        f.close()
        return (major, minor, micro)

    def generate_version(target, source, env ):
        major, minor, micro = read_version(source[0].srcnode().abspath)
        tr = string.maketrans('.-/', '___')
        f = open(target[0].abspath, 'w')
        guard = target[0].path.translate(tr).upper()
        f.write('#ifndef __%s__\n' % (guard))
        f.write('#define __%s__\n\n' % (guard))
        f.write('#define VERSION_MAJOR %s\n' % (major))
        f.write('#define VERSION_MINOR %s\n' % (minor))
        f.write('#define VERSION_MICRO %s\n' % (micro))
        f.write('\n#endif')
        f.close()
        return 0

    def pseudo_version_tuple(env, source):
        return read_version(source.srcnode().abspath)

    def pseudo_version_string(env, source):
        return '.'.join(read_version(source.srcnode().abspath))

    env['VERSIONCOMSTR'] = 'VERSION  $TARGET'
    if env['V']:
        env['VERSIONCOMSTR'] = ''

    env.Append(BUILDERS={
        'Version': SCons.Builder.Builder(
                  action=SCons.Action.Action(generate_version, '$VERSIONCOMSTR'),
                  suffix='.h',
        ),
    })

    env.AddMethod(pseudo_version_tuple, 'VersionTuple')
    env.AddMethod(pseudo_version_string, 'VersionString')

def generate(env, **kwargs):
    add_builders(env)

