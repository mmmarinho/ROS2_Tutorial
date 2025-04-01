Create the package
==================

The first step is to create the package with the correct dependencies. In this example we depend on
:program:`sas_core`, :program:`sas_common`, and :program:`sas_robot_driver`. Using :program:`ros2 pkg create`
we do

.. code-block:: console

  cd ~/sas_tutorial_workspace/src
  ros2 pkg create sas_robot_driver_myrobot \
  --build-type ament_cmake \
  --dependencies rclcpp sas_core sas_common sas_robot_driver

which outputs

.. dropdown:: ros2 pkg create output

    .. code-block:: console
        :emphasize-lines: 1-2
    
        going to create a new package
        package name: sas_robot_driver_myrobot
        destination directory: /home/murilo/Downloads/pycharm-community-2024.3.5/bin
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['murilo <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_cmake
        dependencies: ['rclcpp', 'sas_core', 'sas_common', 'sas_robot_driver']
        creating folder ./sas_robot_driver_myrobot
        creating ./sas_robot_driver_myrobot/package.xml
        creating source and include folder
        creating folder ./sas_robot_driver_myrobot/src
        creating folder ./sas_robot_driver_myrobot/include/sas_robot_driver_myrobot
        creating ./sas_robot_driver_myrobot/CMakeLists.txt

        [WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
        It is recommended to use one of the ament license identifiers:
        Apache-2.0
        BSL-1.0
        BSD-2.0
        BSD-2-Clause
        BSD-3-Clause
        GPL-3.0-only
        LGPL-3.0-only
        MIT
        MIT-0

Package-related sources
-----------------------

.. admonition:: In this step, we'll work on these.

    .. code-block:: console
        :emphasize-lines: 2,6

        └── sas_robot_driver_myrobot
            ├── CMakeLists.txt
            ├── include
            │   └── sas_robot_driver_myrobot
            ├── package.xml
            └── src

The files already exist, we just need to modify them as follows

.. tab-set::

    .. tab-item:: package.xml

        :download:`package.xml <../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/package.xml>`

        .. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/package.xml
           :language: xml
           :linenos:
           :emphasize-lines: 10,18

    .. tab-item:: CMakeLists.txt

        :download:`CMakeLists.txt <../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/CMakeLists.txt>`

        .. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/CMakeLists.txt
           :language: cmake
           :linenos:
           :emphasize-lines: 12-47


Making your own ``sas`` robot drivers
-------------------------------------

.. admonition:: (Murilo's) ``sas_robot_driver`` best practices

   For each new robot called ``myrobot`` we have

   #. :file:`include/sas_robot_driver_myrobot/sas_robot_driver_myrobot.hpp` with the driver's class definition that that inherits from ``sas_robot_driver``. This file must not include any internal driver or library files because it will be exported.
   #. :file:`src/sas_robot_driver_myrobot.cpp` with the driver's class implementation. Any internal libraries or drivers must be included here so that they are not externally visible.
   #. :file:`src/sas_robot_driver_myrobot_node.cpp`.

Create all relevant files

.. code-block:: console

  cd ~/sas_tutorial_workspace/src/sas_robot_driver_myrobot
  mkdir -p src
  touch src/sas_robot_driver_myrobot.cpp
  touch src/sas_robot_driver_myrobot_node.cpp
  mkdir -p include/sas_robot_driver_myrobot
  touch include/sas_robot_driver_myrobot/sas_robot_driver_myrobot.hpp
  mkdir -p launch
  touch launch/real_robot_launch.py

Contents of the header file
---------------------------

