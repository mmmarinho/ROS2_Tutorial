Creating a Python Node with a template (for :code:`ament_python`)
=================================================================

It is always good to rely on the templates available in :program:`ros2 pkg create`, mostly because the best practices for packaging might change between ROS2 versions. 

Let us use the template for creating a package with a Node, as follows.

.. code:: bash

   cd ~/ros2_tutorial_workspace/src
   ros2 pkg create --build-type ament_python python_package_with_a_node --node-name sample_python_node
   
Which will output many things in common with the prior example, but two major differences. 

#. It generates a template Node :code:`creating ./python_package_with_a_node/python_package_with_a_node/sample_python_node.py`.
#. The :file:`setup.py` has information about the Node.

.. code-block:: bash
   :emphasize-lines: 16, 25

   going to create a new package
   package name: python_package_with_a_node
   destination directory: ~/ros2_tutorial_workspace/src
   package format: 3
   version: 0.0.0
   description: TODO: Package description
   maintainer: ['murilo <murilomarinho@ieee.org>']
   licenses: ['TODO: License declaration']
   build type: ament_python
   dependencies: []
   node_name: sample_python_node
   creating folder ./python_package_with_a_node
   creating ./python_package_with_a_node/package.xml
   creating source folder
   creating folder ./python_package_with_a_node/python_package_with_a_node
   creating ./python_package_with_a_node/setup.py
   creating ./python_package_with_a_node/setup.cfg
   creating folder ./python_package_with_a_node/resource
   creating ./python_package_with_a_node/resource/python_package_with_a_node
   creating ./python_package_with_a_node/python_package_with_a_node/__init__.py
   creating folder ./python_package_with_a_node/test
   creating ./python_package_with_a_node/test/test_copyright.py
   creating ./python_package_with_a_node/test/test_flake8.py
   creating ./python_package_with_a_node/test/test_pep257.py
   creating ./python_package_with_a_node/python_package_with_a_node/sample_python_node.py

   [WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
   It is recommended to use one of the ament license identitifers:
   Apache-2.0
   BSL-1.0
   BSD-2.0
   BSD-2-Clause
   BSD-3-Clause
   GPL-3.0-only
   LGPL-3.0-only
   MIT
   MIT-0

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

.. include:: the_canonical_build_command.rst
