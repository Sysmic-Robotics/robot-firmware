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


import unittest

from moteus_gui.python_console_ast import has_loops, inject_yields_in_loops


def _wrap_async(source):
    """Replicate tview's async-exec wrapping so we can compile-check
    transformed sources end-to-end."""
    indented = '\n'.join('    ' + line for line in source.splitlines())
    return f"async def _async_exec():\n{indented}\n    return None"


class HasLoopsTest(unittest.TestCase):
    def test_top_level_for(self):
        self.assertTrue(has_loops("for x in items:\n    print(x)\n"))

    def test_top_level_while(self):
        self.assertTrue(has_loops("while True:\n    pass\n"))

    def test_no_loop(self):
        self.assertFalse(has_loops("x = 1\nprint(x)\n"))

    def test_loop_inside_sync_def(self):
        self.assertFalse(has_loops(
            "def f():\n"
            "    for x in items:\n"
            "        print(x)\n"))

    def test_loop_inside_async_def(self):
        self.assertFalse(has_loops(
            "async def f():\n"
            "    for x in items:\n"
            "        await something(x)\n"))

    def test_loop_inside_class_method(self):
        self.assertFalse(has_loops(
            "class C:\n"
            "    def m(self):\n"
            "        for x in items:\n"
            "            print(x)\n"))

    def test_loop_inside_lambda_via_comprehension(self):
        # A lambda can't directly contain a `for` statement, but a
        # comprehension at module scope creates a hidden `For` node
        # under a Lambda-like scope.  has_loops should ignore those
        # too — comprehensions aren't `For` AST nodes anyway.
        self.assertFalse(has_loops("xs = [x*x for x in items]\n"))

    def test_syntax_error_returns_false(self):
        self.assertFalse(has_loops("def (\n"))


class InjectYieldsTest(unittest.TestCase):
    def test_top_level_for_gets_yield(self):
        out = inject_yields_in_loops("for x in items:\n    print(x)\n")
        self.assertIn('await asyncio.sleep(0)', out)
        compile(_wrap_async(out), '<test>', 'exec')

    def test_top_level_while_gets_yield(self):
        out = inject_yields_in_loops("while True:\n    do_thing()\n")
        self.assertIn('await asyncio.sleep(0)', out)
        compile(_wrap_async(out), '<test>', 'exec')

    def test_loop_inside_sync_def_left_alone(self):
        # Regression: previously injected `await` into the sync body,
        # producing `SyntaxError: 'await' outside async function`
        # when the wrapped source was compiled.
        src = (
            "def all_ids():\n"
            "    result = []\n"
            "    for d in controllers:\n"
            "        result.append(d)\n"
            "    return result\n"
        )
        out = inject_yields_in_loops(src)
        self.assertNotIn('await asyncio.sleep(0)', out)
        compile(_wrap_async(out), '<test>', 'exec')

    def test_loop_inside_class_method_left_alone(self):
        src = (
            "class C:\n"
            "    def m(self):\n"
            "        for x in items:\n"
            "            print(x)\n"
        )
        out = inject_yields_in_loops(src)
        self.assertNotIn('await asyncio.sleep(0)', out)
        compile(_wrap_async(out), '<test>', 'exec')

    def test_loop_inside_async_def_left_alone(self):
        src = (
            "async def f():\n"
            "    for x in items:\n"
            "        await something(x)\n"
        )
        out = inject_yields_in_loops(src)
        self.assertEqual(out.count('await asyncio.sleep(0)'), 0)
        compile(_wrap_async(out), '<test>', 'exec')

    def test_top_level_loop_with_inner_sync_def(self):
        # Mixed case: a top-level loop containing a sync helper that
        # also has a loop.  Only the outer loop should be rewritten.
        src = (
            "for x in items:\n"
            "    def helper():\n"
            "        for y in x:\n"
            "            print(y)\n"
            "    helper()\n"
        )
        out = inject_yields_in_loops(src)
        self.assertEqual(out.count('await asyncio.sleep(0)'), 1)
        compile(_wrap_async(out), '<test>', 'exec')

    def test_syntax_error_returns_source(self):
        bad = "def (\n"
        self.assertEqual(inject_yields_in_loops(bad), bad)


if __name__ == '__main__':
    unittest.main()
