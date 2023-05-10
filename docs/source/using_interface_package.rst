Using interfaces from another package
=====================================

.. code:: console

  ros2 pkg create python_package_that_uses_the_interfaces \
  --build-type ament_python \
  --dependencies rclpy package_with_interfaces
  

.. code:: console

  going to create a new package
  package name: python_package_that_uses_the_interfaces
  destination directory: /home/murilo/git/ROS2_Tutorial/ros2_tutorial_workspace/src
  package format: 3
  version: 0.0.0
  description: TODO: Package description
  maintainer: ['murilo <murilomarinho@ieee.org>']
  licenses: ['TODO: License declaration']
  build type: ament_python
  dependencies: ['rclpy', 'package_with_interfaces']
  creating folder ./python_package_that_uses_the_interfaces
  creating ./python_package_that_uses_the_interfaces/package.xml
  creating source folder
  creating folder ./python_package_that_uses_the_interfaces/python_package_that_uses_the_interfaces
  creating ./python_package_that_uses_the_interfaces/setup.py
  creating ./python_package_that_uses_the_interfaces/setup.cfg
  creating folder ./python_package_that_uses_the_interfaces/resource
  creating ./python_package_that_uses_the_interfaces/resource/python_package_that_uses_the_interfaces
  creating ./python_package_that_uses_the_interfaces/python_package_that_uses_the_interfaces/__init__.py
  creating folder ./python_package_that_uses_the_interfaces/test
  creating ./python_package_that_uses_the_interfaces/test/test_copyright.py
  creating ./python_package_that_uses_the_interfaces/test/test_flake8.py
  creating ./python_package_that_uses_the_interfaces/test/test_pep257.py

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




