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


import asyncio
import unittest

from moteus.async_timeout import timeout


class AsyncTimeoutTest(unittest.IsolatedAsyncioTestCase):
    async def test_single_timeout_fires(self):
        with self.assertRaises(asyncio.TimeoutError):
            async with timeout(0.01):
                await asyncio.sleep(10)

    async def test_no_timeout_when_under_deadline(self):
        async with timeout(10):
            await asyncio.sleep(0.001)

    async def test_none_delay_disables_timeout(self):
        async with timeout(None):
            await asyncio.sleep(0.001)

    async def test_outer_fires_first_inner_does_not_misclaim(self):
        # Outer deadline expires while the inner block is still well
        # within its own deadline.  The inner block must let the
        # cancellation propagate, and only the outer block should
        # report a TimeoutError.
        inner_caught = False
        outer_caught = False
        try:
            async with timeout(0.01):
                try:
                    async with timeout(10):
                        await asyncio.sleep(10)
                except asyncio.TimeoutError:
                    inner_caught = True
        except asyncio.TimeoutError:
            outer_caught = True

        self.assertFalse(
            inner_caught,
            "inner Timeout misattributed outer cancel as its own")
        self.assertTrue(
            outer_caught,
            "outer Timeout failed to fire after inner block exited")

    async def test_external_cancel_not_claimed_as_timeout(self):
        # A cancellation issued from outside the Timeout block must
        # propagate as CancelledError, not be converted to TimeoutError.
        async def victim():
            async with timeout(10):
                await asyncio.sleep(10)

        task = asyncio.create_task(victim())
        # Let the task start and arm its timeout.
        await asyncio.sleep(0.001)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task


if __name__ == '__main__':
    unittest.main()
