#!/usr/bin/env python3
"""Tests for murilos_ros2_linter.py: python3 test_murilos_ros2_linter.py"""

import sys
import tempfile
from pathlib import Path

from murilos_ros2_linter import ERROR, WARNING, lint_package, lint_source

TEMPLATE = """
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class N(Node):
    pass

def main(args=None):
    try:
        rclpy.init(args=args)
        node = N()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(e)
"""

SPLIT_EXCEPTS = """
import rclpy
from rclpy.executors import ExternalShutdownException

def main():
    try:
        rclpy.init()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
"""

QUALIFIED_NAME = """
import rclpy

def main():
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
"""

NESTED_FINALLY = """
import rclpy

def main():
    try:
        try:
            rclpy.spin(node)
        finally:
            cleanup()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
"""

RERAISE_TO_OUTER = """
import rclpy

def main():
    try:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            raise
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
"""

EXECUTOR_SPIN = """
import rclpy

def main():
    executor = rclpy.executors.SingleThreadedExecutor()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
"""

NO_TRY = """
import rclpy

def main():
    rclpy.init()
    rclpy.spin(node)
"""

OLD_TUTORIAL = """
import rclpy

def main():
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
"""

EXCEPTION_FIRST = """
import rclpy

def main():
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
"""

HANDLER_LOGS = """
import rclpy

def main():
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        get_logger().info("bye")
"""

BARE_EXCEPT = """
import rclpy

def main():
    try:
        rclpy.spin(node)
    except:
        pass
"""

NAMING_OK = """
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

class AmazingQuotePublisher(Node):
    def __init__(self):
        super().__init__("amazing_quote_publisher")
        self.pub = self.create_publisher(String, "/amazing_quote", 10)
        self.create_timer(0.5, self.timer_callback)
    def timer_callback(self):
        self.pub.publish(String(data="hello"))

def main(args=None):
    try:
        rclpy.init()
        node = AmazingQuotePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
"""

NAMING_BAD = """
import rclpy
from rclpy.node import Node

class my_node(Node):
    def __init__(self):
        super().__init__("My Node")
        self.pub = self.create_publisher(String, "my-topic", 10)
    def DoStuff(self):
        pass

def MakeIt():
    pass
"""

PRACTICES_OK = """
from python_package_with_a_node.sample_module import Thing
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

class PublisherNode(Node):
    \"\"\"A well-behaved publisher.\"\"\"
    def __init__(self):
        super().__init__("publisher_node")
        self.declare_parameter("period", 0.5)
        self.period = self.get_parameter("period") \\
            .get_parameter_value().double_value
        self.create_timer(self.period, self.timer_callback)
    def timer_callback(self) -> None:
        \"\"\"Log one line.\"\"\"
        self.get_logger().info("hi")

def main(args=None) -> None:
    \"\"\"Entry point.\"\"\"
    try:
        rclpy.init()
        node = PublisherNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
"""

PRACTICES_BAD = """
import rclpy
from rclpy.node import Node
from .sibling_module import Thing

class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher")
        self.period = self.get_parameter("period") \\
            .get_parameter_value().double_value
        print("started")
    def timer_callback(self, extra: int):
        pass

def main(args=None):
    node = PublisherNode()
    rclpy.init()
    rclpy.spin(node)
    rclpy.shutdown()
"""

SYNTAX_ERROR = "def main(:\n"


PKG_XML = ('<?xml version="1.0"?>\n'
           '<package format="3"><name>{}</name></package>\n')
NODE_MAIN = """import rclpy
from rclpy.node import Node

class Barker(Node):
    def __init__(self):
        super().__init__("barker")

def main(args=None):
    pass
"""


def make_pkg(root, folder="demo", xml_name="demo",
             scripts=("barker = demo.barker:main",),
             files=None, resource=True, setup_body=None):
    pkg = Path(root) / folder
    (pkg / "demo").mkdir(parents=True)
    (pkg / "package.xml").write_text(PKG_XML.format(xml_name))
    (pkg / "demo" / "__init__.py").write_text("")
    for rel, content in (files or {"demo/barker.py": NODE_MAIN}).items():
        target = pkg / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    if setup_body is None:
        setup_body = (
            "from setuptools import setup\n"
            "setup(\n    name='demo',\n"
            "    entry_points={'console_scripts': [\n"
            + "".join(f"        '{s}',\n" for s in scripts)
            + "    ]},\n)\n")
    (pkg / "setup.py").write_text(setup_body)
    if resource:
        (pkg / "resource").mkdir(exist_ok=True)
        (pkg / "resource" / xml_name).write_text("")
    return pkg


PKG_CASES = [
    ("good", {}, []),
    ("folder_mismatch", {"folder": "demoo"},
     [(ERROR, "must match")]),
    ("unlisted_node",
     {"files": {"demo/barker.py": NODE_MAIN,
                "demo/sleeper.py": NODE_MAIN.replace("Barker", "Sleeper")}},
     [(ERROR, "no console_scripts entry")]),
    ("missing_module",
     {"scripts": ("barker = demo.barker:main",
                  "ratter = demo.ratter:main")},
     [(ERROR, "points at module")]),
    ("missing_func",
     {"scripts": ("barker = demo.barker:startup",)},
     [(ERROR, "targets function")]),
    ("no_resource", {"resource": False}, [(WARNING, "resource/")]),
    ("dynamic_setup",
     {"setup_body":
      "from setuptools import setup\n"
      "package_name = 'demo'\n"
      "setup(name='demo', entry_points={'console_scripts': ["
      "package_name + '.barker:main']})\n"}, []),
]


def run_pkg_cases():
    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        for i, (label, kwargs, expected) in enumerate(PKG_CASES):
            base = Path(tmp) / f"case{i}"
            base.mkdir()
            pkg = make_pkg(base, **kwargs)
            findings = [(sev, msg) for _, sev, msg in lint_package(pkg)]
            ok = len(findings) == len(expected) and all(
                sev == exp_sev and needle in msg
                for (sev, msg), (exp_sev, needle) in zip(findings, expected))
            print(f"pkg {i:2d} {label:16s}: {'ok' if ok else 'FAIL'}")
            if not ok:
                failures += 1
                print(f"    expected: {expected}")
                print(f"    got:      {findings}")
    return failures


def run(source, path="sample_python_node.py", require_node_suffix=False,
        require_docstrings=False, require_type_hints=False):
    return [(sev, msg) for _, sev, msg in
            lint_source(source, path, require_node_suffix,
                        require_docstrings, require_type_hints)]


CASES = [
    (TEMPLATE, None, False, []),
    (SPLIT_EXCEPTS, None, False, []),
    (QUALIFIED_NAME, None, False, []),
    (NESTED_FINALLY, None, False, []),
    (RERAISE_TO_OUTER, None, False, []),
    (EXECUTOR_SPIN, None, False, []),
    (NO_TRY, None, False, [(ERROR, "not wrapped")]),
    (OLD_TUTORIAL, None, False,
     [(WARNING, "caught only by a broad except Exception")]),
    (EXCEPTION_FIRST, None, False,
     [(WARNING, "caught only by a broad except Exception")]),
    (HANDLER_LOGS, None, False, [(WARNING, "does more than"),
                                 (WARNING, "does more than")]),
    (BARE_EXCEPT, None, False, [(WARNING, "caught only by a bare except"),
                                (WARNING, "caught only by a bare except")]),
    (NAMING_OK, None, False, []),
    (NAMING_BAD, None, False, [(WARNING, "should be PascalCase"),
                               (WARNING, "node name 'My Node'"),
                               (WARNING, "'my-topic'"),
                               (WARNING, "'DoStuff'"),
                               (WARNING, "'MakeIt'")]),
    (NAMING_OK, "bad_name.py", False, []),
    (NAMING_OK, "bad_name.py", True,
     [(WARNING, "should be named *_node.py")]),
    (NAMING_OK, "bad_name_node.py", True, []),
    (PRACTICES_OK, None, (False, True, True), []),
    (PRACTICES_BAD, None, False,
     [(WARNING, "from .sibling_module import"),
      (WARNING, "get_parameter('period') without"),
      (WARNING, "print() inside a Node subclass"),
      (ERROR, "not wrapped"),
      (WARNING, "shutdown() is not in the tutorial template")]),
    (PRACTICES_BAD, None, (False, True, True),
     [(WARNING, "from .sibling_module import"),
      (WARNING, "class 'PublisherNode' has no docstring"),
      (WARNING, "get_parameter('period') without"),
      (WARNING, "print() inside a Node subclass"),
      (WARNING, "'timer_callback' has no docstring"),
      (WARNING, "partial type hints"),
      (WARNING, "'main' has no docstring"),
      (ERROR, "not wrapped"),
      (WARNING, "shutdown() is not in the tutorial template")]),
    (SYNTAX_ERROR, None, False, [(ERROR, "could not parse")]),
]


def main():
    failures = 0
    for i, (source, path, suffix, expected) in enumerate(CASES):
        flags = suffix if isinstance(suffix, tuple) else (suffix,)
        findings = run(source, path or "sample_python_node.py", *flags)
        ok = len(findings) == len(expected) and all(
            sev == exp_sev and needle in msg
            for (sev, msg), (exp_sev, needle) in zip(findings, expected))
        print(f"case {i:2d}: {'ok' if ok else 'FAIL'}")
        if not ok:
            failures += 1
            print(f"    expected: {expected}")
            print(f"    got:      {findings}")
    failures += run_pkg_cases()
    print("FAILED" if failures else "all tests passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
