Parameters: Launching configurable nodes
========================================

The Nodes we have made in the past few sections are interesting because they take advantage of the interprocess communication provided by ROS2. 

  Other capabilities of ROS2 that we must take advantage of are `ROS2 parameters <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html>`_
and `ROS2 launch files <https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html>`_. We can use them to modify the behavior of Nodes without having
  to modify their source code. For Python users, that might sound less appealing than users of compiled languages. However, users of your package might not want nor be able
  to modify the source code directly, if the package is installable or part of a larger system with multiple users.

  
Create the package
------------------

First, let us create an :program:`ament_python` package that depends on our :file:`packages_with_interfaces` and build from there.

.. code:: console

  cd ~/ros2_tutorial_workspace/src
  ros2 pkg create python_package_that_uses_parameters_and_launch_files \
  --build-type ament_python \
  --dependencies rclpy package_with_interfaces


Overview
--------

Before we start exploring the elements of the package, let us

#. Create the Node with a configurable publisher using parameters, mostly as we saw in :ref:`Create a publisher`.
#. Create a launch file to configure the Node without modifying its source code.

Create the Node using parameters
--------------------------------

.. admonition:: **TL:DR** Using parameters in a Node

               #. Declare the parameter with :code:`Node.declare_parameter()`, usually in the class's :code:`__init__`.
               #. Get the parameter with :code:`Node.get_parameter()` either once or continuously

For the sake of the example, let us suppose that we want to make an :code:`AmazingQuote` publisher that is, now, configurable.

Let's start by creating an :file:`amazing_quote_configurable_publisher_node.py` in :file:`~/ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/python_package_that_uses_parameters_and_launch_files` with the following contents

:download:`amazing_quote_configurable_publisher_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/python_package_that_uses_parameters_and_launch_files/amazing_quote_configurable_publisher_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/python_package_that_uses_parameters_and_launch_files/amazing_quote_configurable_publisher_node.py
   :language: python
   :lineno:
   :lines: 24-
