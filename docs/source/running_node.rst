Running a node (:program:`ros2 run`)
====================================

The most basic way of running a Node is using the ROS2 tool :program:`ros2 run`.

More information on it can be obtained through

.. code:: bash

   ros2 run -h

Back to our example, with a properly sourced terminal, the example node can be executed with

.. code:: bash

   ros2 run python_package_with_a_node sample_python_node

which will now correctly output

.. code:: bash

   Hi from python_package_with_a_node.
