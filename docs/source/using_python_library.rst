Using a Python Library (for :code:`ament_python`)
=================================================

Let us create a package with a Node that uses the library we created in the prior example. 

Note that we must now add it as a dependency when calling :program:`ros2 pkg create`. We also add :code:`rclcpp` as a dependency so that we can do something useful with it.

.. code:: bash

   cd ~/ros2_tutorial_workspace/src
   ros2 pkg create python_package_that_uses_the_library --build-type ament_python --node-name node_that_uses_the_library --dependencies rclcpp python_package_with_a_library
   
resulting in yet another version of our favorite wall of text

.. code-block:: console
   :emphasize-lines: 27

   going to create a new package
   package name: python_package_that_uses_the_library
   destination directory: /home/murilo/git/ROS2_Tutorial/ros2_tutorial_workspace/src
   package format: 3
   version: 0.0.0
   description: TODO: Package description
   maintainer: ['murilo <murilomarinho@ieee.org>']
   licenses: ['TODO: License declaration']
   build type: ament_python
   dependencies: ['rclcpp', 'python_package_with_a_library']
   node_name: node_that_uses_the_library
   creating folder ./python_package_that_uses_the_library
   creating ./python_package_that_uses_the_library/package.xml
   creating source folder
   creating folder ./python_package_that_uses_the_library/python_package_that_uses_the_library
   creating ./python_package_that_uses_the_library/setup.py
   creating ./python_package_that_uses_the_library/setup.cfg
   creating folder ./python_package_that_uses_the_library/resource
   creating ./python_package_that_uses_the_library/resource/python_package_that_uses_the_library
   creating ./python_package_that_uses_the_library/python_package_that_uses_the_library/__init__.py
   creating folder ./python_package_that_uses_the_library/test
   creating ./python_package_that_uses_the_library/test/test_copyright.py
   creating ./python_package_that_uses_the_library/test/test_flake8.py
   creating ./python_package_that_uses_the_library/test/test_pep257.py
   creating ./python_package_that_uses_the_library/python_package_that_uses_the_library/node_that_uses_the_library.py

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