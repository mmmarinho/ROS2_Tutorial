Creating a dedicated package for messages
=========================================

All interfaces in ROS2 must be made in an :program:`ament_cmake` package. We have so far not needed it, but for this scenario we cannot escape. Thankfully, for this we don't need to dig too deep into :program:`CMake` just for this purpose, so fear not.

Creating the package
--------------------

There isn't a template for message-only packages using :program:`ros2 pkg create`. We'll need to build on top of a mostly empty :program:`ament_cmake` package.

Run

.. code:: console

   cd ~/ros2_tutorial_workspace/src
   ros2 pkg create --build-type ament_cmake package_with_interfaces

again showing our beloved wall of text, with a few highlighted differences because of it being a :program:`ament_cmake` package.

.. code-block:: console
    :emphasize-lines: 9, 14, 15, 16

    going to create a new package
    package name: package_with_interfaces
    destination directory: /home/murilo/git/ROS2_Tutorial/ros2_tutorial_workspace/src
    package format: 3
    version: 0.0.0
    description: TODO: Package description
    maintainer: ['murilo <murilomarinho@ieee.org>']
    licenses: ['TODO: License declaration']
    build type: ament_cmake
    dependencies: []
    creating folder ./package_with_interfaces
    creating ./package_with_interfaces/package.xml
    creating source and include folder
    creating folder ./package_with_interfaces/src
    creating folder ./package_with_interfaces/include/package_with_interfaces
    creating ./package_with_interfaces/CMakeLists.txt

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

The :file:`package.xml` works the same way as when using :file:`ament_python`. However, we no longer have a :file:`setup.py` or :file:`setup.cfg`, everything is handled by the :file:`CMakeLists.txt`.

The :file:`package.xml` must include extra dependencies
-------------------------------------------------------

Whenever the package has any type of interface, the :file:`package.xml` **must** include three specific dependencies. Namely,

:download:`package.xml <../../ros2_tutorial_workspace/src/package_with_interfaces/package.xml>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/package.xml
   :language: xml
   :linenos:
   :emphasize-lines: 12,13,14





