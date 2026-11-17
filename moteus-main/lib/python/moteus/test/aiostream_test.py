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
import threading
import unittest

from moteus.aiostream import AioStream


class _EofFd:
    """Minimal file-like object that always returns b'' (EOF)."""

    def __init__(self):
        self.read_count = 0
        self._lock = threading.Lock()

    def read(self, n):
        with self._lock:
            self.read_count += 1
        return b''


class _ScriptedFd:
    """File-like object returning a queued sequence of byte responses."""

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self._lock = threading.Lock()

    def read(self, n):
        with self._lock:
            if not self._chunks:
                return b''
            chunk = self._chunks.pop(0)
            return chunk[:n]


class _RaisingFd:
    """File-like object whose read/write raises a fixed exception."""

    def __init__(self, exc):
        self._exc = exc

    def read(self, n):
        raise self._exc

    def write(self, data):
        raise self._exc


class AioStreamTest(unittest.IsolatedAsyncioTestCase):
    async def test_blocking_read_returns_on_eof(self):
        # Regression test: previously a blocking read on an fd that
        # returned b'' would loop forever, pegging CPU.  After the fix
        # it must return what it has (here: nothing) within the
        # timeout.
        fd = _EofFd()
        stream = AioStream(fd)

        result = await asyncio.wait_for(
            stream.read(10, block=True), timeout=2.0)

        self.assertEqual(result, b'')
        # The worker should not have spun: a single read call is
        # plenty.  Allow a little slack for thread-scheduling jitter.
        self.assertLess(
            fd.read_count, 100,
            f'fd.read called {fd.read_count} times — busy loop suspected')

    async def test_blocking_read_short_then_eof(self):
        # If we get a partial read followed by EOF, the blocking read
        # must return the partial bytes rather than spinning.
        fd = _ScriptedFd([b'abc'])
        stream = AioStream(fd)

        result = await asyncio.wait_for(
            stream.read(10, block=True), timeout=2.0)

        self.assertEqual(result, b'abc')

    async def test_nonblocking_read_returns_partial(self):
        # Non-blocking still returns whatever the fd produced on the
        # first call, including EOF.
        fd = _EofFd()
        stream = AioStream(fd)

        result = await asyncio.wait_for(
            stream.read(10, block=False), timeout=2.0)

        self.assertEqual(result, b'')

    async def test_read_propagates_fd_exception(self):
        # Regression test: previously fd.read() raising left the
        # awaiting coroutine hung forever (the worker thread died and
        # never resolved the future).  The exception must now reach
        # the caller, and a subsequent read attempt must also fail
        # (not hang) because the worker thread is still alive.
        #
        # We use a custom exception class rather than e.g. OSError
        # because asyncio.wait_for's TimeoutError is itself an
        # OSError on modern Python, which would mask a hang.
        class _Boom(Exception):
            pass

        fd = _RaisingFd(_Boom('simulated I/O error'))
        stream = AioStream(fd)

        with self.assertRaises(_Boom):
            await asyncio.wait_for(stream.read(10), timeout=2.0)

        # Second attempt should also raise quickly, not hang —
        # confirms the worker thread is still alive.
        with self.assertRaises(_Boom):
            await asyncio.wait_for(stream.read(10), timeout=2.0)

    async def test_drain_propagates_fd_exception(self):
        class _Boom(Exception):
            pass

        fd = _RaisingFd(_Boom('peer closed'))
        stream = AioStream(fd)

        stream.write(b'hello')
        with self.assertRaises(_Boom):
            await asyncio.wait_for(stream.drain(), timeout=2.0)

        stream.write(b'world')
        with self.assertRaises(_Boom):
            await asyncio.wait_for(stream.drain(), timeout=2.0)


if __name__ == '__main__':
    unittest.main()
