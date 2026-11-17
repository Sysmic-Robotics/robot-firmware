#!/usr/bin/python3

# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Capture data from a fdcanusb to a file'''

import sys
import time

fd = open(sys.argv[1])
# Line-buffered so data already captured survives an abrupt
# termination (USB unplug, SIGINT, etc.).
out = open(sys.argv[2], "w", buffering=1)
while True:
    line = fd.readline()
    if not line:
        # EOF: input descriptor closed (USB device unplugged, regular
        # file fully read, peer side of a pipe closed).  readline()
        # returns '' indefinitely after this, so we'd otherwise
        # busy-spin filling the output with timestamp-only lines.
        break
    print(f'{time.time():.6f} {line.rstrip()}', file=out)
