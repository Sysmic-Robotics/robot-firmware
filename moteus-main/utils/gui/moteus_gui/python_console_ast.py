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

"""AST helpers for the tview Python REPL.

The REPL wraps the user's snippet in `async def _async_exec(): ...` so
that top-level `await` is supported and tight loops can be made
cancellable.  These helpers decide whether such wrapping is needed and
inject `await asyncio.sleep(0)` at the start of any loop body that
would benefit.

Crucially, the injection only applies at the wrapper's own scope.
Loops inside a nested `def`, `async def`, `class`, or `lambda` are
left alone: `await` is illegal inside a sync function body, async
functions manage their own yields, and class bodies execute
synchronously.
"""

import ast


_OPAQUE_SCOPES = (ast.FunctionDef, ast.AsyncFunctionDef,
                  ast.ClassDef, ast.Lambda)


def has_loops(source):
    """Return True if @p source has a `for`/`while` at the wrapper
    scope (i.e. not inside a nested function, class, or lambda)."""
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return False

    def walk(node):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.While, ast.For)):
                return True
            if isinstance(child, _OPAQUE_SCOPES):
                continue
            if walk(child):
                return True
        return False

    return walk(tree)


class _LoopTransformer(ast.NodeTransformer):
    # Don't recurse into nested function or class bodies.  `await` is
    # only legal in the generated async wrapper frame; injecting it
    # inside a sync `def`, an `async def` that already manages its
    # own yields, a `class` body, or a `lambda` would either be a
    # SyntaxError or stomp on the user's intent.
    def visit_FunctionDef(self, node):
        return node

    def visit_AsyncFunctionDef(self, node):
        return node

    def visit_ClassDef(self, node):
        return node

    def visit_Lambda(self, node):
        return node

    def _inject(self, node):
        self.generic_visit(node)
        yield_stmt = ast.Expr(
            value=ast.Await(
                value=ast.Call(
                    func=ast.Attribute(
                        value=ast.Name(id='asyncio', ctx=ast.Load()),
                        attr='sleep',
                        ctx=ast.Load()),
                    args=[ast.Constant(value=0)],
                    keywords=[])))
        node.body = [yield_stmt] + node.body
        return node

    visit_For = _inject
    visit_While = _inject


def inject_yields_in_loops(source):
    """Insert `await asyncio.sleep(0)` at the start of every
    wrapper-scope loop body in @p source so the event loop has a
    chance to process cancellation between iterations.

    On any AST manipulation failure, returns @p source unchanged.
    """
    try:
        tree = ast.parse(source)
        new_tree = _LoopTransformer().visit(tree)
        ast.fix_missing_locations(new_tree)
        return ast.unparse(new_tree)
    except Exception:
        return source
