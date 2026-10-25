#!/usr/bin/env python3
"""Lint rclpy.spin() call sites against the ROS2_Tutorial shutdown template.

The tutorial (``python_node_explained`` and the week 3-5 lecture decks)
requires every ``rclpy.spin()`` to sit inside::

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(e)

On ROS 2 Jazzy rclpy installs its own SIGINT handler and shuts the context
down itself, so ``spin()`` raises ``ExternalShutdownException`` (a plain
``except KeyboardInterrupt`` never fires) and a bare ``except Exception``
only swallows that one by accident -- which is not the same as handling
Ctrl-C correctly.

For every spin call site this linter checks that, following normal Python
first-match handler semantics and propagating through enclosing tries when
handlers re-raise or do not match:

  errors
    * spin is not guarded by any try/except at all;
    * KeyboardInterrupt or ExternalShutdownException escapes to the module
      top (unhandled -> traceback on Ctrl-C);
    * ExternalShutdownException is caught only by a broad ``except
      Exception`` (it should be caught explicitly with KeyboardInterrupt).
  warnings
    * the operative handler is a bare ``except:``;
    * the operative handler does more than ``pass``.

Known limitation: a spin() that is wrapped lexically but called from a try
elsewhere (``def run(): rclpy.spin(n)``) is reported as unguarded -- the
template calls spin inside main()'s try directly.

Usage::

    ./lint_spin_template.py [paths ...] [--strict] [--quiet]

Paths default to the current directory; directories are scanned for *.py.
Exit code: 1 when errors (or, with --strict, warnings) were found.

Test: ``python3 test_lint_spin_template.py``
"""

import argparse
import ast
import sys
from pathlib import Path

SPIN_FUNCS = {"spin", "spin_once", "spin_until_future_complete"}
SKIP_DIRS = {"install", "build", "log", "__pycache__", ".venv",
             ".git", ".idea", "node_modules"}
EXCEPTIONS = ("KeyboardInterrupt", "ExternalShutdownException")

ERROR = "error"
WARNING = "warning"


def _is_spin(call: ast.Call) -> bool:
    func = call.func
    if isinstance(func, ast.Name):
        return func.id in SPIN_FUNCS
    if not isinstance(func, ast.Attribute) or func.attr not in SPIN_FUNCS:
        return False
    base = func.value
    if isinstance(base, ast.Name):
        return base.id == "rclpy" or "executor" in base.id.lower()
    if isinstance(base, ast.Attribute):
        return base.attr == "executors"
    return False


def _name(node: ast.expr) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return ""


def _covers(handler: ast.ExceptHandler, exc: str):
    """'explicit' if handler names exc, 'broad' if Exception/bare catches
    it, None otherwise. KeyboardInterrupt derives from BaseException, so
    `except Exception` does not catch it."""
    if handler.type is None:
        return "broad"
    names = ([_name(t) for t in handler.type.elts]
             if isinstance(handler.type, ast.Tuple) else [_name(handler.type)])
    if exc in names:
        return "explicit"
    if exc != "KeyboardInterrupt" and "Exception" in names:
        return "broad"
    return None


def _reises(handler: ast.ExceptHandler) -> bool:
    return any(isinstance(n, ast.Raise) for n in ast.walk(handler))


def _just_pass(handler: ast.ExceptHandler) -> bool:
    return len(handler.body) == 1 and isinstance(handler.body[0], ast.Pass)


def _parents(tree: ast.AST) -> dict:
    return {child: node for node in ast.walk(tree)
            for child in ast.iter_child_nodes(node)}


def _operative_handler(exc, spin, parents):
    """First-match handler for exc across enclosing tries, or None if exc
    escapes to the module top. Handlers that re-raise let it propagate."""
    cur = spin
    while True:
        node = parents.get(cur)
        if node is None:
            return None
        if (isinstance(node, ast.Try) and cur in node.body
                and node.handlers):
            for handler in node.handlers:
                kind = _covers(handler, exc)
                if kind is None:
                    continue
                if _reises(handler):
                    break  # re-raised: keep looking in enclosing tries
                return handler, kind
        cur = node


def lint_source(source: str, path: str):
    try:
        tree = ast.parse(source, filename=path)
    except SyntaxError as err:
        return [(err.lineno or 1, ERROR, f"could not parse: {err.msg}")]
    parents = _parents(tree)
    findings = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and _is_spin(node)):
            continue
        guarded = any(isinstance(n, ast.Try) and n.handlers
                      for n in _ancestors(node, parents))
        if not guarded:
            findings.append((node.lineno, ERROR,
                             "rclpy.spin() is not wrapped in try/except "
                             "(or is called from outside the try; see "
                             "--help for the expected template)"))
            continue
        for exc in EXCEPTIONS:
            found = _operative_handler(exc, node, parents)
            if found is None:
                findings.append((node.lineno, ERROR,
                                 f"{exc} escapes to the module top: an "
                                 "unhandled traceback on Ctrl-C costs the "
                                 "clean-shutdown mark"))
                continue
            handler, kind = found
            if kind == "broad" and handler.type is None:
                findings.append((handler.lineno,
                                 ERROR if exc == "ExternalShutdownException"
                                 else WARNING,
                                 f"bare except swallows everything; catch "
                                 f"{exc} explicitly" if exc == "KeyboardInterrupt"
                                 else "ExternalShutdownException is caught "
                                 "only by a bare except -- catch it "
                                 "explicitly with KeyboardInterrupt"))
            elif kind == "broad":
                findings.append((handler.lineno, ERROR,
                                 "ExternalShutdownException is caught only "
                                 "by a broad except Exception -- catch it "
                                 "explicitly: "
                                 "except (KeyboardInterrupt, "
                                 "ExternalShutdownException): pass"))
            elif not _just_pass(handler):
                findings.append((handler.lineno, WARNING,
                                 f"handler for {exc} does more than 'pass' "
                                 "(template uses pass)"))
    return findings


def _ancestors(node, parents):
    cur = node
    while True:
        cur = parents.get(cur)
        if cur is None:
            return
        yield cur


def iter_python_files(paths):
    for p in (Path(x) for x in paths):
        if p.is_file():
            yield p
        elif p.is_dir():
            for f in sorted(p.rglob("*.py")):
                if not (set(f.parts) & SKIP_DIRS or f.name.startswith(".")):
                    yield f


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.split("Usage::")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="*", default=["."])
    parser.add_argument("--strict", action="store_true",
                        help="treat warnings as errors")
    parser.add_argument("--quiet", action="store_true",
                        help="only print the summary")
    args = parser.parse_args(argv)

    n_err = n_warn = n_files = n_spins = 0
    for path in iter_python_files(args.paths):
        try:
            source = path.read_text(encoding="utf-8", errors="replace")
        except OSError as err:
            print(f"{path}:0: error: {err}", file=sys.stderr)
            n_err += 1
            continue
        n_files += 1
        for lineno, severity, message in lint_source(source, str(path)):
            n_err += severity == ERROR
            n_warn += severity == WARNING
            if not args.quiet:
                print(f"{path}:{lineno}: {severity}: {message}")
    if not args.quiet or n_err or (args.strict and n_warn):
        print(f"{n_files} file(s), {n_err} error(s), {n_warn} warning(s)")
    if n_err or (args.strict and n_warn):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
