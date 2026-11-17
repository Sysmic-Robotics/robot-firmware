#!/usr/bin/python3 -B

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

"""Small helpers for working with encoder positions expressed in
turns (a value of 1.0 == one full revolution).  Shared by the
host-side encoder calibration / comparison utilities."""

import math
import numpy


def wrap_half(value):
    """Reduce @p value into (-0.5, 0.5] turns."""
    while value > 0.5:
        value -= 1.0
    while value < -0.5:
        value += 1.0
    return value


def wrap_zero_one(value):
    """Reduce @p value into [0.0, 1.0) turns."""
    while value > 1.0:
        value -= 1.0
    while value < 0.0:
        value += 1.0
    return value


def circular_mean(values):
    """Mean of a set of angles expressed in turns, returned in
    [-0.5, 0.5].  Avoids the bunching-near-the-wrap-boundary failure
    that a plain arithmetic mean has when samples cluster near 0/1
    (or, equivalently, +/-0.5)."""
    angles = numpy.asarray(values) * 2.0 * math.pi
    s = numpy.mean(numpy.sin(angles))
    c = numpy.mean(numpy.cos(angles))
    return math.atan2(s, c) / (2.0 * math.pi)
