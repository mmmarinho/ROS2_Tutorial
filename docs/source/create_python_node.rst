Creating a Python Node (using :code:`ament_python`)
===================================================

Create a package with a Node template
-------------------------------------

.. code:: bash

   cd ~/ros2_tutorial_workspace/src
   ros2 pkg create --build-type ament_python python_package_with_a_node --dependencies rclpy --node-name sample_python_node
   
Which will output many things in common with the prior example (removed for readability), but with two additional tasks automatically performed. First, it adds the :code:`dependencies: ['rclpy']` for us. Then, it generates a template node :code:`creating ./python_package_with_a_node/python_package_with_a_node/sample_python_node.py`.

.. code:: bash

    going to create a new package
    package name: python_package_with_a_node
    (...)
    dependencies: ['rclpy']
    (...)
    creating ./python_package_with_a_node/python_package_with_a_node/sample_python_node.py
    (...)

Then, we can build the workspace as usual to consider the new package as well.

.. code:: bash

   cd ~/ros2_tutorial_workspace
   colcon build
   
which will result in going through the package we created in the prior example and the current one.   

.. code:: bash

    Starting >>> python_package_with_a_node
    Starting >>> the_simplest_python_package
    --- stderr: python_package_with_a_node                                   
    /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
      warnings.warn(
    ---
    Finished <<< python_package_with_a_node [1.16s]
    --- stderr: the_simplest_python_package
    /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
      warnings.warn(
    ---
    Finished <<< the_simplest_python_package [1.17s]

    Summary: 2 packages finished [1.36s]
      2 packages had stderr output: python_package_with_a_node the_simplest_python_package
      
Always source after you build
-----------------------------

When creating new packages or modifying existing ones, many changes will not be visible by the system unless our workspace is re-sourced.

For example, if we try the following in the terminal window we used to first build this example package

.. code:: bash

   ros2 run python_package_with_a_node sample_python_node

it will not work and will output

.. code:: bash

   Package 'python_package_with_a_node' not found
   
As the workspace grows bigger and the packages more complex, figuring out such errors becomes a considerable hassle. One suggestion is to always source after a build, so that sourcing errors can always be ruled out.

.. code:: bash

   cd ~/ros2_tutorial_workspace
   colcon build
   source install/setup.bash
   
Running a node
--------------

With a properly sourced terminal, the example node can be executed with

.. code:: bash

   ros2 run python_package_with_a_node sample_python_node

which will now correctly output

.. code:: bash

   Hi from python_package_with_a_node.
