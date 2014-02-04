# Copyright (c) 2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import signal
from subprocess import Popen, PIPE, STDOUT


## Simple function to execute a linux shell command
#  @param command The bash command to be run
#  @param quiet Tells the function no to print the output of the command
#  @param get_output Tells the script to return the output of the command
def mk_process(command, quiet=False, get_output=False, shell=True):
    if shell == True:
        process = Popen(command, shell=True, stdout=PIPE,
                               stderr=STDOUT)
    else:
        process = Popen(command, shell=False, stdout=PIPE,
                               stderr=STDOUT)
    stdout, stderr = process.communicate()
    if quiet == False:
        print stdout
    if get_output:
        return stdout
    else:
        return process.returncode


# Pretty much the same as mk_process, but in a class so that
#   the user can call the access the process after instantiation
class RosProcess():
    def __init__(self, command, quiet=True, get_output=False, shell=True):
        if shell == True:
            self.process = Popen(command, shell=True, stdout=None,
                                        stdin=PIPE, stderr=STDOUT)
        else:
            self.process = Popen(command, shell=False, stdout=None,
                                        stdin=PIPE, stderr=STDOUT)
        self.quiet = quiet
        self.get_output = get_output

    def write(self, stdin):
        self.process.stdin.write(stdin)

    def run(self):
        stdout, stderr = self.process.communicate()
        if self.quiet == False:
            print stdout
        if self.get_output:
            return stdout
        else:
            return self.process.returncode


def python_processes():
    return [p for p in mk_process('ps ax',
                                  get_output=True,
                                  quiet=True).split('\n') if 'python' in p]


def python_proc_ids(proc):
    return [int(p.split()[0]) for p in python_processes() if proc in p]


def kill_python_procs(proc):
    for idx in python_proc_ids(proc):
        os.kill(idx, signal.SIGINT)
