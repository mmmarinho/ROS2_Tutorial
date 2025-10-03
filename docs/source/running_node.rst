.. _Running a node:

Running a node (:program:`ros2 run`)
====================================

The most basic way of running a Node is using the ROS2 tool :program:`ros2 run`.

More information on it can be obtained through

.. code :: console

   ros2 run -h
   
which returns the most relevant arguments :code:`package_name` and :code:`executable_name`.

.. code-block:: console
   :emphasize-lines: 6, 7

   usage: ros2 run [-h] [--prefix PREFIX] package_name executable_name ...

   Run a package specific executable

   positional arguments:
     package_name     Name of the ROS package
     executable_name  Name of the executable
     argv             Pass arbitrary arguments to the executable

   options:
     -h, --help       show this help message and exit
     --prefix PREFIX  Prefix command, which should go before the executable. Command must be wrapped in quotes if it contains spaces (e.g. --prefix 'gdb -ex run --args').


Back to our example, with a properly sourced terminal, the example node can be executed with

.. code :: console

   ros2 run python_package_with_a_node sample_python_node

which will now correctly output

.. code :: console

   Hi from python_package_with_a_node.

Troubleshooting tips
--------------------

If :program:`ROS2` is unable to find the node, but it is able to find the package, then you can rely on :program:`ros2 pkg executables`. For instance,
you can run as follows.

.. code :: console

    ros2 pkg executables python_package_with_a_node

The command, at this stage, should output the following.

.. code :: console

    python_package_with_a_node sample_python_node

This shows that :program:`ROS2` has been correctly able to find :file:`sample_python_node` within :file:`python_package_with_a_node`.
If the command outputs nothing, this means that no nodes were found.