#!/usr/bin/env python3
"""Murilo's ROS 2 linter: spin template, naming and package structure checks.

Shutdown template (``python_node_explained``, week 3-5 lecture decks):
every ``rclpy.spin()`` must sit inside::

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
      top (unhandled -> traceback on Ctrl-C).
  warnings
    * KeyboardInterrupt or ExternalShutdownException is caught only by a
      broad ``except Exception``/bare ``except:`` -- the old tutorial
      style: it works (exit 0, no KeyboardInterrupt traceback) but relies
      on swallowing, so the explicit tuple is preferred;
    * the operative handler is not just ``pass``;
    * a spin function is handed to something else as a callback/thread
      target (``threading.Thread(target=rclpy.spin, args=(node,))``): there
      is no ``Call`` node for that spin, so the template cannot be checked
      here -- the fix is to wrap it in a function of your own that uses the
      template, which this linter can then check.

Best practices (``python_best_practices``, ``python_node_explained``,
``parameters_and_launch``) -- all warnings:

    * no relative imports (absolute ``from <package>.<module> import``);
    * no ``destroy_node()``/``rclpy.shutdown()`` (not in the template);
    * ``get_logger().info()`` instead of ``print()`` inside Node
      subclasses;
    naming warnings are skipped when the name is not the author's to
    choose -- Qt's virtuals/event handlers (``QT_OVERRIDES``:
    ``paintEvent``, ``setupUi``, ...), the API a test double mirrors
    (``MIRROR_NAMES``: ``isOpened`` of ``cv2.VideoCapture``) and modules
    that declare themselves tool-generated -- because warning about names
    nobody picked teaches people to ignore the linter;
    * ``get_parameter('x')`` only after ``declare_parameter('x', ...)``.
    With --require-docstrings: classes/public functions need docstrings.
    With --require-type-hints: functions with partial type hints are
    flagged (annotate all parameters and the return).

Naming (``python_best_practices`` and the node template) -- all warnings:

    * class names PascalCase;
    * function/method names snake_case (PEP8);
    * the node name passed to ``super().__init__()`` snake_case;
    * topic/service names passed to create_publisher/subscription/
      service/client snake_case (string literals only).
    With --require-node-suffix additionally warns when a module defining a
    Node subclass is not named *_node.py (the tutorial's own convention;
    leave it off if your project names modules differently).

Package structure checks run for every package.xml found under the given
paths (lint_package() is importable for zip-based flows):

  errors
    * package folder name != <name> in package.xml;
    * console_scripts entry point pointing at a missing module or a
      missing target function;
    * a console_scripts entry not of the form 'name =
      package.module:function' (tutorial 5.1 format; a missing comma
      silently fuses two entries);
    * a module in the package defining a Node subclass and main() but not
      listed in console_scripts (ros2 run cannot launch it);
    * a Node module importing a package that package.xml does not declare
      as <depend> (tutorial 5.1 "Handling dependencies");
    * the <package>/<package>/ Python folder without __init__.py (tutorial
      5.1 file structure; ros2 run cannot import the modules).
  warnings
    * module with main() but no entry point (no Node subclass);
    * missing resource/<package_name> marker file;
    * non-Node module importing an undeclared third-party top-level
      module.

Known limitation: a spin() that is wrapped lexically but called from a try
elsewhere (``def run(): rclpy.spin(n)``) is reported as unguarded -- the
template calls spin inside main()'s try directly.

Programmatic API (importable, no I/O besides reading the given files):
``lint_source(source, path, ...)`` and ``lint_package(pkg_dir)`` return
(lineno, severity, message) findings; ``shutdown_template_status(source,
path)`` returns per-spin-site dicts (guarded, keyboard/shutdown caught
explicitly or by a broad handler, has_broad_handler, ...) so scripts can
check the shutdown template of a source programmatically instead of
reading findings: conformant is guarded, with both exceptions caught
(explicitly or by the broad handler) and a broad ``except Exception``
present -- the two styles the tutorial teaches.

Usage::

    ./murilos_ros2_linter.py [paths ...] [--strict] [--quiet]
        [--require-node-suffix] [--require-docstrings]
        [--require-type-hints]

Paths default to the current directory; directories are scanned for *.py.
Exit code: 1 when errors (or, with --strict, warnings) were found.

Test: ``python3 test_murilos_ros2_linter.py``
"""

import argparse
import ast
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SPIN_FUNCS = {"spin", "spin_once", "spin_until_future_complete"}
TOPIC_NAME_FUNCS = {"create_publisher", "create_subscription",
                    "create_service", "create_client"}
SKIP_DIRS = {"install", "build", "log", "__pycache__", ".venv",
             ".git", ".idea", "node_modules"}
EXCEPTIONS = ("KeyboardInterrupt", "ExternalShutdownException")

## Qt's own virtuals and event handlers, plus the two members Qt Designer
## generates. Renaming them is not an option -- Qt calls them by name -- so
## the naming checks skip them (the tutorial's snake_case rule is about code
## a student writes, not about satisfying a C++ library's meta-object).
QT_OVERRIDES = frozenset({
    "setupUi", "retranslateUi",
    "paintEvent", "paint", "initStyleOption", "hitTest",
    "sizeHint", "minimumSizeHint", "maximumSizeHint", "sizeIncrement",
    "event", "eventFilter", "changeEvent", "actionEvent",
    "showEvent", "hideEvent", "closeEvent", "resizeEvent", "moveEvent",
    "keyPressEvent", "keyReleaseEvent", "focusInEvent", "focusOutEvent",
    "enterEvent", "leaveEvent", "hoverEnterEvent", "hoverMoveEvent",
    "hoverLeaveEvent", "wheelEvent", "contextMenuEvent", "tabletEvent",
    "mousePressEvent", "mouseReleaseEvent", "mouseMoveEvent",
    "mouseDoubleClickEvent", "dragEnterEvent", "dragMoveEvent",
    "dragLeaveEvent", "dropEvent", "touchEvent",
})

## Names a *test double* has to reproduce verbatim: ``camera/tests/conftest.py``
## fakes ``cv2.VideoCapture``, whose API is camelCase. Not a style choice by
## the author, so not a finding -- same reason as ``QT_OVERRIDES``.
MIRROR_NAMES = frozenset({"isOpened", "getTickFrequency", "waitKey"})

## Files nobody hand-wrote. Their names come from a tool (pyuic, grpc, a
## message generator), so warning about them trains people to ignore the
## linter; the spin template is still checked inside them.
_GENERATED_MARKERS = ("generated by", "generated from", "auto-generated",
                      "autogenerated", "do not edit", "do not modify",
                      "@generated", "created by:", "will be lost")

PASCAL_RE = re.compile(r"_?[A-Z][A-Za-z0-9]*$")
SNAKE_RE = re.compile(r"_{0,2}[a-z][a-z0-9_]*$")
NODE_SUFFIXES = {"Node"}

ERROR = "error"
WARNING = "warning"


def _spin_ref(expr: ast.expr):
    """The spin function ``expr`` refers to (``spin``, ``rclpy.spin``,
    ``ex.spin``, ``rclpy.executors.spin``...), or None. Shared by the call
    sites below and by the callback form, where the name is used without
    being called."""
    if isinstance(expr, ast.Name):
        return expr.id if expr.id in SPIN_FUNCS else None
    if not isinstance(expr, ast.Attribute) or expr.attr not in SPIN_FUNCS:
        return None
    base = expr.value
    if isinstance(base, ast.Name):
        if base.id == "rclpy" or "executor" in base.id.lower():
            return expr.attr
        return None
    if isinstance(base, ast.Attribute):
        return expr.attr if base.attr == "executors" else None
    return None


def _is_spin(call: ast.Call) -> bool:
    return _spin_ref(call.func) is not None


def _spin_callback(call: ast.Call):
    """The spin passed as an argument (``Thread(target=rclpy.spin,
    args=(node,))``, ``functools.partial(rclpy.spin, node)``), or None.

    Such a spin has no ``Call`` node, so ``lint_source`` never reaches it
    with the template check and reports nothing while Ctrl-C prints a
    traceback from the worker thread. Only a warning: the handler belongs to
    rclpy's own body, which this file cannot inspect.
    """
    for expr in list(call.args) + [kw.value for kw in call.keywords]:
        if _spin_ref(expr) is not None:
            return expr
    return None


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


def _is_node_subclass(node: ast.ClassDef) -> bool:
    for base in node.bases:
        name = _name(base)
        if name in NODE_SUFFIXES or name.endswith("Node"):
            return True
    return False


def _node_name_arg(classdef: ast.ClassDef):
    """The string literal passed to super().__init__('name') in this class."""
    for sub in ast.walk(classdef):
        if (isinstance(sub, ast.Call)
                and isinstance(sub.func, ast.Attribute)
                and sub.func.attr == "__init__"
                and isinstance(sub.func.value, ast.Call)
                and isinstance(sub.func.value.func, ast.Name)
                and sub.func.value.func.id == "super"
                and sub.args and isinstance(sub.args[0], ast.Constant)
                and isinstance(sub.args[0].value, str)):
            return sub.args[0]
    return None


def _snake_ok(name: str) -> bool:
    return bool(SNAKE_RE.match(name.lstrip("/~").rstrip("/")))


def _name_is_forced(name: str) -> bool:
    """True when some other library picked the name (Qt virtuals, an API a
    test double mirrors), so snake_case is not an option."""
    return name in QT_OVERRIDES or name in MIRROR_NAMES


def _is_generated(tree: ast.AST, head_lines=()) -> bool:
    """True when the module declares itself tool-generated (docstring or the
    first comments), e.g. Qt Designer's ui_mainwindow.py."""
    head = (ast.get_docstring(tree) or "") + "\n" + "\n".join(list(head_lines)[:20])
    return any(marker in head.lower() for marker in _GENERATED_MARKERS)


def _lint_names(tree: ast.AST, head_lines=()):
    """Naming conventions taught in the tutorial (python_best_practices:
    PascalCase classes, snake_case files and functions; snake_case node,
    topic and service names)."""
    findings = []
    generated = _is_generated(tree, head_lines)
    exempt = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and any(
                _name(b).endswith("TestCase") for b in node.bases):
            for sub in node.body:  # unittest API forces camelCase names
                if isinstance(sub, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    exempt.add(id(sub))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            if not generated and not PASCAL_RE.match(node.name):
                findings.append((node.lineno, WARNING,
                                 f"class name '{node.name}' should be "
                                 "PascalCase (tutorial: naming of classes)"))
            if _is_node_subclass(node):
                arg = _node_name_arg(node)
                if arg is not None and not _snake_ok(arg.value):
                    findings.append((arg.lineno, WARNING,
                                     f"node name '{arg.value}' should be "
                                     "snake_case (template uses e.g. "
                                     "sample_python_node)"))
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if id(node) not in exempt and not node.name.startswith("__") \
                    and not generated and not _name_is_forced(node.name) \
                    and not SNAKE_RE.match(node.name.lstrip("_")):
                findings.append((node.lineno, WARNING,
                                 f"function/method name '{node.name}' "
                                 "should be snake_case (tutorial: naming of "
                                 "functions, PEP8)"))
        elif (isinstance(node, ast.Call)
              and isinstance(node.func, ast.Attribute)
              and node.func.attr in TOPIC_NAME_FUNCS
              and len(node.args) >= 2
              and isinstance(node.args[1], ast.Constant)
              and isinstance(node.args[1].value, str)
              and "{" not in node.args[1].value):
            name = node.args[1].value
            if name and not all(_snake_ok(seg)
                                for seg in name.lstrip("/~").split("/")):
                findings.append((node.lineno, WARNING,
                                 f"{node.func.attr} name '{name}' should be "
                                 "snake_case"))
    return findings


def _lint_best_practices(tree: ast.AST, require_docstrings: bool,
                         require_type_hints: bool):
    """Assorted MUSTs/shoulds from python_best_practices,
    python_node_explained and parameters_and_launch."""
    findings = []
    node_classes = [n for n in ast.walk(tree)
                    if isinstance(n, ast.ClassDef) and _is_node_subclass(n)]
    inside_node = {id(sub) for c in node_classes for sub in ast.walk(c)}
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.level:
            names = ", ".join(a.name for a in node.names)
            findings.append((node.lineno, WARNING,
                             f"relative import 'from {'.' * node.level}"
                             f"{node.module or ''} import {names}': the "
                             "tutorial teaches absolute "
                             "'from <package>.<module> import ...' (relative "
                             "imports break in deployed packages)"))
        elif isinstance(node, ast.Call):
            if isinstance(node.func, ast.Attribute) and (
                    node.func.attr == "destroy_node"
                    or (node.func.attr == "shutdown"
                        and isinstance(node.func.value, ast.Name)
                        and node.func.value.id == "rclpy")):
                findings.append((node.lineno, WARNING,
                                 f"{node.func.attr}() is not in the tutorial "
                                 "template: rclpy shuts the context down "
                                 "itself, and adding it crashes on Ctrl-C "
                                 "and hides the 0 exit code"))
            elif (isinstance(node.func, ast.Name)
                  and node.func.id == "print" and id(node) in inside_node):
                findings.append((node.lineno, WARNING,
                                 "print() inside a Node subclass: use "
                                 "self.get_logger().info() (tutorial: The "
                                 "Python Node, explained)"))
    for c in node_classes:
        declared, used = set(), []
        for sub in ast.walk(c):
            if (isinstance(sub, ast.Call)
                    and isinstance(sub.func, ast.Attribute)
                    and sub.func.attr in ("declare_parameter",
                                          "get_parameter")
                    and sub.args and isinstance(sub.args[0], ast.Constant)
                    and isinstance(sub.args[0].value, str)):
                if sub.func.attr == "declare_parameter":
                    declared.add(sub.args[0].value)
                else:
                    used.append((sub.args[0].value, sub.lineno))
        for pname, lineno in used:
            if pname not in declared:
                findings.append((lineno, WARNING,
                                 f"get_parameter('{pname}') without "
                                 f"declare_parameter('{pname}') in class "
                                 f"'{c.name}': get_parameter fails on "
                                 "undeclared parameters"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            if require_docstrings and ast.get_docstring(node) is None:
                findings.append((node.lineno, WARNING,
                                 f"class '{node.name}' has no docstring "
                                 "(tutorial: Document your code with "
                                 "Docstrings)"))
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if require_docstrings and not node.name.startswith("_") \
                    and ast.get_docstring(node) is None:
                findings.append((node.lineno, WARNING,
                                 f"method/function '{node.name}' has no "
                                 "docstring (tutorial: Document your code "
                                 "with Docstrings)"))
            if require_type_hints:
                params = [p for p in
                          node.args.posonlyargs + node.args.args
                          + node.args.kwonlyargs
                          if p.arg not in ("self", "cls")]
                annotated = [p for p in params if p.annotation is not None]
                if annotated and (len(annotated) < len(params)
                                  or node.returns is None):
                    findings.append((node.lineno, WARNING,
                                     f"'{node.name}' has partial type hints: "
                                     "annotate all parameters and the return "
                                     "(tutorial: Take the (type) hint)"))
    return findings


def lint_source(source: str, path: str, require_node_suffix: bool = False,
                require_docstrings: bool = False,
                require_type_hints: bool = False):
    try:
        tree = ast.parse(source, filename=path)
    except SyntaxError as err:
        return [(err.lineno or 1, ERROR, f"could not parse: {err.msg}")]
    parents = _parents(tree)
    findings = _lint_names(tree, source.splitlines())
    findings += _lint_best_practices(tree, require_docstrings,
                                     require_type_hints)
    if require_node_suffix and any(
            isinstance(n, ast.ClassDef) and _is_node_subclass(n)
            for n in ast.walk(tree)):
        base = Path(path).name
        if not base.endswith("_node.py") and base != "__init__.py":
            findings.append((1, WARNING,
                             f"module '{base}' defines a Node subclass and "
                             "should be named *_node.py (tutorial "
                             "convention)"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if _is_spin(node):
            findings.extend(_lint_spin_call(node, parents))
            continue
        target = _spin_callback(node)
        if target is not None:
            findings.append((node.lineno, WARNING,
                             f"{ast.unparse(target)} is passed to "
                             f"{ast.unparse(node.func)}() as a callback/thread"
                             " target: that spin runs outside any try in this "
                             "file, so wrap it in a function using the "
                             "template (except (KeyboardInterrupt, "
                             "ExternalShutdownException): pass)"))
    return sorted(findings, key=lambda f: f[0])


def _lint_spin_call(node: ast.Call, parents: dict):
    """Template findings for one spin() call site."""
    findings = []
    guarded = any(isinstance(n, ast.Try) and n.handlers
                  for n in _ancestors(node, parents))
    if not guarded:
        findings.append((node.lineno, ERROR,
                         "rclpy.spin() is not wrapped in try/except "
                         "(or is called from outside the try; see "
                         "--help for the expected template)"))
        return findings
    for exc in EXCEPTIONS:
        found = _operative_handler(exc, node, parents)
        if found is None:
            findings.append((node.lineno, ERROR,
                             f"{exc} escapes to the module top: Ctrl-C "
                             "would print a traceback and exit "
                             "non-zero (see the template in --help)"))
            continue
        handler, kind = found
        if kind == "broad":
            style = ("a bare except" if handler.type is None
                     else "a broad except Exception")
            if exc == "KeyboardInterrupt":
                findings.append((handler.lineno, WARNING,
                                 f"KeyboardInterrupt is caught only by "
                                 f"{style}; catch it explicitly"))
            else:
                findings.append((handler.lineno, WARNING,
                                 "ExternalShutdownException is caught "
                                 f"only by {style}: old tutorial style, "
                                 "it works (exit 0, no KeyboardInterrupt "
                                 "traceback) but relies on swallowing; "
                                 "prefer except (KeyboardInterrupt, "
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


def shutdown_template_status(source: str, path: str = "<string>"):
    """Per-spin() call site template status, for scripts that need to know
    whether a source follows the shutdown template without reading the
    findings as text (or running a live Ctrl-C).

    Returns one dict per spin call site (same set of sites the template
    checks above report on), each with:

        lineno             line of the spin call
        parsed             False if the module does not parse (reason set)
        guarded            a try/except with handlers encloses the spin
        keyboard           'explicit' | 'broad' | None (escapes to top)
                           for KeyboardInterrupt
        shutdown           same for ExternalShutdownException
        has_broad_handler  an enclosing try also has a non-reraising
                           ``except Exception``/bare handler (the template's
                           ``except Exception as e: print(e)``)
        handlers_pass      every operative handler is just ``pass``
        callback_form      spin is a thread/callback target, uninspectable

    A site is template-conformant when ``guarded`` and both ``keyboard``
    and ``shutdown`` are caught and ``has_broad_handler`` holds: exactly
    the two styles the tutorial teaches --
    ``except (KeyboardInterrupt, ExternalShutdownException)`` +
    ``except Exception``, or the old ``except KeyboardInterrupt`` +
    ``except Exception`` (which catches ESE by swallowing).
    """
    try:
        tree = ast.parse(source, filename=path)
    except SyntaxError as err:
        return [{"lineno": err.lineno or 1, "parsed": False,
                 "guarded": False, "keyboard": None, "shutdown": None,
                 "has_broad_handler": False, "handlers_pass": False,
                 "callback_form": False,
                 "reason": f"could not parse: {err.msg}"}]
    parents = _parents(tree)
    statuses = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if _is_spin(node):
            guarded = any(isinstance(n, ast.Try) and n.handlers
                          for n in _ancestors(node, parents))
            kinds, handlers = {}, []
            for exc in EXCEPTIONS:
                found = (_operative_handler(exc, node, parents)
                         if guarded else None)
                kinds[exc] = found[1] if found else None
                if found:
                    handlers.append(found[0])
            has_broad = any(
                (handler.type is None
                 or "Exception" in ([_name(t) for t in handler.type.elts]
                                    if isinstance(handler.type, ast.Tuple)
                                    else [_name(handler.type)]))
                and not _reises(handler)
                for anc in _ancestors(node, parents)
                if isinstance(anc, ast.Try)
                for handler in anc.handlers)
            statuses.append({
                "lineno": node.lineno, "parsed": True, "guarded": guarded,
                "keyboard": kinds["KeyboardInterrupt"],
                "shutdown": kinds["ExternalShutdownException"],
                "has_broad_handler": has_broad,
                "handlers_pass": bool(handlers)
                and all(_just_pass(h) for h in handlers),
                "callback_form": False})
            continue
        target = _spin_callback(node)
        if target is not None:
            statuses.append({"lineno": node.lineno, "parsed": True,
                             "guarded": False, "keyboard": None,
                             "shutdown": None, "has_broad_handler": False,
                             "handlers_pass": False, "callback_form": True})
    return statuses


def iter_python_files(paths):
    for p in (Path(x) for x in paths):
        if p.is_file():
            yield p
        elif p.is_dir():
            for f in sorted(p.rglob("*.py")):
                if not (set(f.parts) & SKIP_DIRS or f.name.startswith(".")):
                    yield f


_CONSOLE_SCRIPT_RE = re.compile(
    r"[A-Za-z0-9._-]+\s*=\s*"
    r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*\s*:\s*[A-Za-z_]\w*\s*")


def _console_script_strings(setup_py: Path):
    """Raw console_scripts entry strings from setup(entry_points=
    {'console_scripts': [...]}) when statically resolvable, else None."""
    try:
        tree = ast.parse(setup_py.read_text(encoding="utf-8",
                                            errors="replace"))
    except (OSError, SyntaxError):
        return None
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and _name(node.func) == "setup"):
            continue
        for kw in node.keywords:
            if kw.arg != "entry_points" or not isinstance(kw.value, ast.Dict):
                continue
            scripts = None
            for k, v in zip(kw.value.keys, kw.value.values):
                if isinstance(k, ast.Constant) and k.value == "console_scripts"\
                        and isinstance(v, (ast.List, ast.Tuple)):
                    if all(isinstance(e, ast.Constant)
                           and isinstance(e.value, str) for e in v.elts):
                        scripts = [e.value for e in v.elts]
            if scripts is None:
                continue
            return scripts
    return None


def _console_scripts(setup_py: Path):
    """[(exe, 'module:func'), ...] from setup(entry_points=
    {'console_scripts': [...]}) when statically resolvable, else None."""
    scripts = _console_script_strings(setup_py)
    if scripts is None:
        return None
    entries = []
    for s in scripts:
        if "=" not in s or ":" not in s:
            continue
        exe, target = s.split("=", 1)
        entries.append((exe.strip(), target.strip()))
    return entries


def _ament_installed():
    """Package names visible in the share/ folders of AMENT_PREFIX_PATH:
    dependencies already installed on this machine (e.g. prebuilt
    interface packages) do not need to be flagged as undeclared."""
    prefixes = [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(
        os.pathsep) if p]
    names = set()
    for prefix in prefixes:
        share = Path(prefix) / "share"
        try:
            names |= {d.name for d in share.iterdir() if d.is_dir()}
        except OSError:
            continue
    return names


def lint_package(pkg_dir: Path):
    """ROS 2 package structure checks (package.xml vs folder, setup.py
    console_scripts vs modules, resource marker). Importable so scripts can
    call it on an unpacked tree (e.g. extracted from a zip)."""
    # resolve(): Path('.').name is '', which would report every package as a
    # folder mismatch when the linter is run from inside the package folder.
    pkg_dir = Path(pkg_dir).resolve()
    findings = []
    try:
        xml_root = ET.parse(pkg_dir / "package.xml").getroot()
    except (OSError, ET.ParseError) as err:
        return [(0, ERROR, f"package.xml could not be parsed: {err}")]
    name = (xml_root.findtext("name") or "").strip()
    if not name:
        findings.append((0, ERROR, "package.xml has no <name>"))
    elif name != pkg_dir.name:
        findings.append((0, ERROR,
                         f"package folder is '{pkg_dir.name}' but "
                         f"package.xml declares <name>{name}</name>: they "
                         "must match (ros2 pkg list/run use the "
                         "declared name)"))
    setup_py = pkg_dir / "setup.py"
    if setup_py.exists():
        for s in _console_script_strings(setup_py) or []:
            if not _CONSOLE_SCRIPT_RE.fullmatch(s):
                findings.append((0, ERROR,
                                 f"malformed console_scripts entry {s!r}: "
                                 "each entry must be one string "
                                 "'name = package.module:function' with a "
                                 "comma between entries (tutorial 5.1 "
                                 "'Making ros2 run work')"))
        entries = _console_scripts(setup_py)
        if entries is not None:
            referenced = set()
            for exe, target in entries:
                mod, _, fn = target.partition(":")
                referenced.add(mod)
                mod_file = pkg_dir.joinpath(*mod.split(".")).with_suffix(".py")
                if not mod_file.exists():
                    findings.append((0, ERROR,
                                     f"entry point '{exe} = {target}' points"
                                     f" at module '{mod}', which has no "
                                     f"{mod.split('.')[-1]}.py in the "
                                     "package"))
                    continue
                if fn and f"def {fn}(" not in mod_file.read_text(
                        encoding="utf-8", errors="replace"):
                    findings.append((0, ERROR,
                                     f"entry point '{exe} = {target}' "
                                     f"targets function '{fn}', which "
                                     f"'{mod_file.name}' does not define"))
            pkg_sub = pkg_dir / name
            if pkg_sub.is_dir():
                for mod_file in sorted(pkg_sub.rglob("*.py")):
                    if mod_file.name == "__init__.py":
                        continue
                    mod = ".".join(mod_file.relative_to(pkg_dir)
                                   .with_suffix("").parts)
                    if mod in referenced:
                        continue
                    try:
                        tree = ast.parse(mod_file.read_text(
                            encoding="utf-8", errors="replace"))
                    except SyntaxError:
                        continue  # reported by the file pass
                    has_main = any(isinstance(n, ast.FunctionDef)
                                   and n.name == "main"
                                   for n in ast.walk(tree))
                    if not has_main:
                        continue
                    has_node = any(isinstance(n, ast.ClassDef)
                                   and _is_node_subclass(n)
                                   for n in ast.walk(tree))
                    findings.append((0, ERROR if has_node else WARNING,
                                     f"'{mod}' defines "
                                     + ("a Node subclass and main() but has "
                                        "no console_scripts entry point: "
                                        "ros2 run cannot launch it"
                                        if has_node else
                                        "main() but has no console_scripts "
                                        "entry point")))
        if name and not (pkg_dir / "resource" / name).exists():
            findings.append((0, WARNING,
                             f"resource/{name} marker file is missing "
                             "(needed by ament_python for ros2 pkg "
                             "discovery)"))
    # tutorial 5.1 (Creating a Python Node from scratch): package.xml
    # declares what the Nodes import; the module folder is a real package.
    declared = {" ".join((t.text or "").split())
                for t in xml_root.iter()
                if t.tag in ("depend", "exec_depend", "build_depend")}
    pkg_sub = pkg_dir / name if name else None
    if pkg_sub is not None and pkg_sub.is_dir():
        if not (pkg_sub / "__init__.py").exists():
            findings.append((0, ERROR,
                             f"{name}/__init__.py is missing: setup.py only "
                             "installs proper Python packages, so ros2 run "
                             "cannot import the modules (tutorial 5.1 file "
                             "structure)"))
        installed = _ament_installed()
        for mod_file in sorted(pkg_sub.rglob("*.py")):
            if mod_file.name == "__init__.py":
                continue
            try:
                tree = ast.parse(mod_file.read_text(
                    encoding="utf-8", errors="replace"))
            except SyntaxError:
                continue  # reported by the file pass
            tops = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    tops.update(a.name.split(".")[0] for a in node.names)
                elif isinstance(node, ast.ImportFrom) and node.level == 0 \
                        and node.module:
                    tops.add(node.module.split(".")[0])
            has_node = any(isinstance(n, ast.ClassDef)
                           and _is_node_subclass(n)
                           for n in ast.walk(tree))
            for top in sorted(tops):
                if (top in sys.stdlib_module_names or top == name
                        or top in declared or top in installed
                        or (pkg_dir / top).exists()
                        or (pkg_dir / (top + ".py")).exists()):
                    continue
                message = (f"'{mod_file.name}' imports '{top}' but "
                           f"package.xml declares no "
                           f"<depend>{top}</depend> (tutorial 5.1 'Handling "
                           f"dependencies': add new dependencies to "
                           f"package.xml)")
                findings.append((0, ERROR if (has_node or top == "rclpy")
                                 else WARNING, message))
    return findings


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.split("Usage::")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="*", default=["."])
    parser.add_argument("--strict", action="store_true",
                        help="treat warnings as errors")
    parser.add_argument("--require-node-suffix", action="store_true",
                        help="warn when a module defining a Node subclass "
                             "is not named *_node.py (the tutorial's own "
                             "convention; leave off if your project names "
                             "modules differently)")
    parser.add_argument("--require-docstrings", action="store_true",
                        help="warn on classes/public functions without "
                             "docstrings (tutorial: Document your code "
                             "with Docstrings)")
    parser.add_argument("--require-type-hints", action="store_true",
                        help="warn on functions with partial type hints "
                             "(tutorial: Take the (type) hint)")
    parser.add_argument("--quiet", action="store_true",
                        help="only print the summary")
    args = parser.parse_args(argv)

    n_err = n_warn = n_files = 0
    for path in iter_python_files(args.paths):
        try:
            source = path.read_text(encoding="utf-8", errors="replace")
        except OSError as err:
            print(f"{path}:0: error: {err}", file=sys.stderr)
            n_err += 1
            continue
        n_files += 1
        for lineno, severity, message in lint_source(
                source, str(path), args.require_node_suffix,
                args.require_docstrings, args.require_type_hints):
            n_err += severity == ERROR
            n_warn += severity == WARNING
            if not args.quiet:
                print(f"{path}:{lineno}: {severity}: {message}")
    for p in (Path(x) for x in args.paths):
        candidates = ([p] if p.name == "package.xml" and p.is_file()
                      else sorted(p.rglob("package.xml")) if p.is_dir()
                      else [])
        for pkg_xml in candidates:
            if set(pkg_xml.parts) & SKIP_DIRS:
                continue
            for lineno, severity, message in lint_package(pkg_xml.parent):
                n_err += severity == ERROR
                n_warn += severity == WARNING
                if not args.quiet:
                    print(f"{pkg_xml.parent}:{lineno}: {severity}: "
                          f"{message}")
    if not args.quiet or n_err or (args.strict and n_warn):
        print(f"{n_files} file(s), {n_err} error(s), {n_warn} warning(s)")
    if n_err or (args.strict and n_warn):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
