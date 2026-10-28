=====
Tools
=====

This tutorial ships a small linter for the practices taught in it:
:file:`tools/murilos_ros2_linter.py`. It is pure standard-library Python,
so there is nothing to install.

It checks four groups of things:

- **Shutdown template**: every :code:`rclpy.spin()` is wrapped according
  to the template of :doc:`The Python Node, explained <python_node/python_node_explained>`,
  i.e., :code:`except (KeyboardInterrupt, ExternalShutdownException): pass`
  before any broad :code:`except Exception`.
- **Naming**: the conventions of :doc:`(Murilo's) Python Best Practices
  <preamble/python/python_best_practices>` — PascalCase classes,
  snake_case functions, node names and topic/service names.
- **Best practices**: no relative imports, no :code:`destroy_node()`/:code:`rclpy.shutdown()`
  (the template omits them), :code:`get_logger()` instead of :code:`print()`
  inside Nodes, and :code:`get_parameter()` only for declared parameters —
  from :doc:`The Python Node, explained <python_node/python_node_explained>`,
  :doc:`Parameters and Launch <parameters_and_launch/parameters_and_launch>` and
  :doc:`(Murilo's) Python Best Practices
  <preamble/python/python_best_practices>`.
- **Package structure**: package folder matches :code:`<name>` in
  :file:`package.xml`, :code:`console_scripts` entry points point at
  existing modules (and are not missing for launchable nodes), and the
  :code:`resource/<package_name>` marker exists.

You can run it on your workspace with a single command, no clone
required:

.. code-block:: console

    curl -fsSL https://raw.githubusercontent.com/mmmarinho/ROS2_Tutorial/main/tools/murilos_ros2_linter.py | python3 - ~/ros2_tutorial_workspace/src

Errors (unguarded :code:`spin()`, escaping shutdown exceptions, broken
entry points, :file:`package.xml` mismatches) make the command exit
non-zero, which is handy in CI or before sharing your code.
Warnings (old-style broad-except shutdown handling, naming, best
practices) are reported but do not fail the run; use :code:`--strict` to
make them fail. Three opt-in checks are off by default:
:code:`--require-node-suffix` (launchable modules must be named
:code:`*_node.py` — leave off if your project names modules differently),
:code:`--require-docstrings` (classes and public functions need
docstrings) and :code:`--require-type-hints` (functions with partial type
hints are flagged). A local copy can be run the same way:

.. code-block:: console

    python3 tools/murilos_ros2_linter.py ~/ros2_tutorial_workspace/src
