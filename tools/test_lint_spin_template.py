#!/usr/bin/env python3
"""Tests for lint_spin_template.py: python3 test_lint_spin_template.py"""

import sys

from lint_spin_template import ERROR, WARNING, lint_source

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

SYNTAX_ERROR = "def main(:\n"


def run(source):
    return [(sev, msg) for _, sev, msg in lint_source(source, "t.py")]


CASES = [
    (TEMPLATE, []),
    (SPLIT_EXCEPTS, []),
    (QUALIFIED_NAME, []),
    (NESTED_FINALLY, []),
    (RERAISE_TO_OUTER, []),
    (EXECUTOR_SPIN, []),
    (NO_TRY, [(ERROR, "not wrapped")]),
    (OLD_TUTORIAL, [(ERROR, "ExternalShutdownException is caught only")]),
    (EXCEPTION_FIRST, [(ERROR, "ExternalShutdownException is caught only")]),
    (HANDLER_LOGS, [(WARNING, "does more than"),
                    (WARNING, "does more than")]),
    (BARE_EXCEPT, [(WARNING, "bare except"),
                   (ERROR, "caught only")]),
    (SYNTAX_ERROR, [(ERROR, "could not parse")]),
]


def main():
    failures = 0
    for i, (source, expected) in enumerate(CASES):
        findings = run(source)
        ok = len(findings) == len(expected) and all(
            sev == exp_sev and needle in msg
            for (sev, msg), (exp_sev, needle) in zip(findings, expected))
        print(f"case {i:2d}: {'ok' if ok else 'FAIL'}")
        if not ok:
            failures += 1
            print(f"    expected: {expected}")
            print(f"    got:      {findings}")
    print("FAILED" if failures else "all tests passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
