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


import math
import unittest

import numpy

from encoder_math import wrap_half, wrap_zero_one, circular_mean


class WrapHalfTest(unittest.TestCase):
    def test_within_range(self):
        for v in (-0.5, -0.25, 0.0, 0.25, 0.5):
            self.assertEqual(wrap_half(v), v)

    def test_above_half(self):
        self.assertAlmostEqual(wrap_half(0.6), -0.4)
        self.assertAlmostEqual(wrap_half(1.25), 0.25)
        self.assertAlmostEqual(wrap_half(2.6), -0.4)

    def test_below_neg_half(self):
        self.assertAlmostEqual(wrap_half(-0.6), 0.4)
        self.assertAlmostEqual(wrap_half(-1.25), -0.25)
        self.assertAlmostEqual(wrap_half(-2.6), 0.4)


class WrapZeroOneTest(unittest.TestCase):
    def test_within_range(self):
        for v in (0.0, 0.25, 0.5, 0.75):
            self.assertEqual(wrap_zero_one(v), v)

    def test_above_one(self):
        self.assertAlmostEqual(wrap_zero_one(1.25), 0.25)
        self.assertAlmostEqual(wrap_zero_one(2.6), 0.6)

    def test_negative(self):
        self.assertAlmostEqual(wrap_zero_one(-0.25), 0.75)
        self.assertAlmostEqual(wrap_zero_one(-1.6), 0.4)


class CircularMeanTest(unittest.TestCase):
    def test_single_value(self):
        for v in (0.0, 0.25, -0.25, 0.5, -0.5):
            self.assertAlmostEqual(circular_mean([v]), wrap_half(v),
                                   places=10)

    def test_constant_input(self):
        # 100 samples all equal — mean should equal that value
        # (modulo wrap to (-0.5, 0.5]).
        for v in (0.0, 0.1, 0.4, -0.1):
            self.assertAlmostEqual(circular_mean([v] * 100), v, places=10)

    def test_returns_in_minus_half_to_half(self):
        # The mean is always reported in the [-0.5, 0.5] interval,
        # regardless of where the input cluster lies.
        for v in (0.0, 0.5, 1.0, 1.5, -0.5, -1.0):
            r = circular_mean([v] * 50)
            self.assertGreaterEqual(r, -0.5)
            self.assertLessEqual(r, 0.5)

    def test_small_offset_matches_arithmetic_mean(self):
        # Far from the wrap boundary, the circular mean reproduces
        # the arithmetic mean.
        numpy.random.seed(0)
        samples = 0.2 + numpy.random.normal(0, 0.001, 200)
        self.assertAlmostEqual(
            circular_mean(samples.tolist()),
            float(numpy.mean(samples)),
            places=4)

    def test_at_wrap_boundary(self):
        # Regression test: arithmetic mean of values bunched near
        # +/-0.5 collapses to ~0; circular mean must instead recover
        # the true offset.
        numpy.random.seed(0)
        samples = 0.5 + numpy.random.normal(0, 0.001, 200)
        # wrap_half'd inputs straddle the +/-0.5 wrap.
        wrapped = [wrap_half(s) for s in samples]
        # Arithmetic mean of the wrapped values is near zero, NOT
        # 0.5 — that's the bug we're protecting against.
        self.assertLess(abs(float(numpy.mean(wrapped))), 0.05)
        # Circular mean recovers the true offset.  +0.5 and -0.5 are
        # the same point on the circle; check magnitude matches.
        self.assertAlmostEqual(abs(circular_mean(samples.tolist())),
                               0.5, places=2)

    def test_sweeps_full_offset_range(self):
        # The estimator must work at every offset, including those
        # near the +/-0.5 boundary.
        numpy.random.seed(0)
        n = 200
        sigma = 0.001
        for true_offset in (0.0, 0.1, 0.3, 0.499, 0.500,
                            -0.499, -0.3, -0.1):
            samples = true_offset + numpy.random.normal(0, sigma, n)
            est = circular_mean(samples.tolist())
            # +0.5 and -0.5 are the same point; compare modulo wrap.
            err = wrap_half(est - true_offset)
            self.assertLess(
                abs(err), 5 * sigma,
                f'true_offset={true_offset} est={est} err={err}')


if __name__ == '__main__':
    unittest.main()
