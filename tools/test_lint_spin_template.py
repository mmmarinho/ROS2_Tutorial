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
    (OLD_TUTORIAL, [(WARNING, "caught only by a broad except Exception")]),
    (EXCEPTION_FIRST, [(WARNING, "caught only by a broad except Exception")]),
    (HANDLER_LOGS, [(WARNING, "does more than"),
                    (WARNING, "does more than")]),
    (BARE_EXCEPT, [(WARNING, "caught only by a bare except"),
                   (WARNING, "caught only by a bare except")]),
    (NAMING_OK, []),
    (NAMING_BAD, [(WARNING, "should be PascalCase"),
                  (WARNING, "node name 'My Node'"),
                  (WARNING, "'my-topic'"),
                  (WARNING, "'DoStuff'"),
                  (WARNING, "'MakeIt'")]),
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
