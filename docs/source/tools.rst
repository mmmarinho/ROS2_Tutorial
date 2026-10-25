=====
Tools
=====

This tutorial ships a small linter for the practices taught in it:
:file:`tools/murilos_ros2_linter.py`. It is pure standard-library Python,
so there is nothing to install.

It checks three groups of things:

- **Shutdown template**: every :code:`rclpy.spin()` is wrapped according
  to the template of :doc:`The Python Node, explained <python_node_explained>`,
  i.e., :code:`except (KeyboardInterrupt, ExternalShutdownException): pass`
  before any broad :code:`except Exception`.
- **Naming**: the conventions of :doc:`(Murilo's) Python Best Practices
  <preamble/python/python_best_practices>` — PascalCase classes,
  snake_case functions, node names and topic/service names.
- **Package structure**: package folder matches :code:`<name>` in
  :file:`package.xml`, :code:`console_scripts` entry points point at
  existing modules (and are not missing for launchable nodes), and the
  :code:`resource/<package_name>` marker exists.

Students can run it on their workspace with a single command, no clone
required:

.. code-block:: console

    curl -fsSL https://raw.githubusercontent.com/mmmarinho/ROS2_Tutorial/main/tools/murilos_ros2_linter.py | python3 - ~/ros2_tutorial_workspace/src

Errors (unguarded :code:`spin()`, escaping shutdown exceptions, broken
entry points, :file:`package.xml` mismatches) make the command exit
non-zero, which is handy before submitting an assessment or in CI.
Warnings (old-style broad-except shutdown handling, naming) are reported
but do not fail the run; use :code:`--strict` to make them fail, and
:code:`--require-node-suffix` to additionally require launchable modules
to be named :code:`*_node.py`. A local copy can be run the same way:

.. code-block:: console

    python3 tools/murilos_ros2_linter.py ~/ros2_tutorial_workspace/src
