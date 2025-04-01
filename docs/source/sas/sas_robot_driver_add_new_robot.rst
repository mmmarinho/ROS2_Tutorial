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
          :emphasize-lines: 2,8

          └── sas_robot_driver_myrobot
              ├── CMakeLists.txt
              ├── include
              │   └── sas_robot_driver_myrobot
              │       └── sas_robot_driver_myrobot.hpp
              ├── launch
              │   └── real_robot_launch.py
              ├── package.xml
              ├── scripts
              │   └── joint_interface_example.py
              └── src
                  ├── sas_robot_driver_myrobot.cpp
                  └── sas_robot_driver_myrobot_node.cpp

The files already exist, we just need to modify them as follows

.. tab-set::

    .. tab-item:: package.xml

        :download:`package.xml <../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/package.xml>`

        .. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/package.xml
           :language: xml
           :linenos:

    .. tab-item:: CMakeLists.txt

        :download:`CMakeLists.txt <../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/CMakeLists.txt>`

        .. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/CMakeLists.txt
           :language: cmake
           :linenos:
           :emphasize-lines: 10,16-132

In :file:`CMakeLists.txt` we have a sequence of four blocks. These are all directly related to ROS2 and although in
this tutorial I define a best practice, this is not particular to ``sas``.




Making your own ``sas`` robot drivers
-------------------------------------

.. admonition:: (Murilo's) ``sas_robot_driver`` best practices

   For each new robot called ``myrobot`` we have the three steps below as a must

   #. :file:`sas_robot_driver_myrobot.hpp` with the driver's class definition that that inherits from ``sas_robot_driver``. This file must not include any internal driver or library files because it will be exported.
   #. :file:`sas_robot_driver_myrobot.cpp` with the driver's class implementation. Any internal libraries or drivers must be included here so that they are not externally visible.
   #. :file:`sas_robot_driver_myrobot_node.cpp` that configures the driver and calls the ROS2 loop.

   The creation of the following two is trivial

   #. :file:`real_robot_launch.py` a suitable launch file to properly configure :file:`sas_robot_driver_myrobot_node.cpp`.
   #. :file:`joint_interface_example.py` a Python script to control the C++ node (if needed).

Let's create all the files used in the remainder of this tutorial.

.. code-block:: console

  cd ~/sas_tutorial_workspace/src/sas_robot_driver_myrobot
  mkdir -p src
  touch src/sas_robot_driver_myrobot.cpp
  touch src/sas_robot_driver_myrobot_node.cpp
  mkdir -p include/sas_robot_driver_myrobot
  touch include/sas_robot_driver_myrobot/sas_robot_driver_myrobot.hpp
  mkdir -p launch
  touch launch/real_robot_launch.py
  mkdir -p scripts
  touch scripts/joint_interface_example.py

The robot driver class
----------------------

.. admonition:: In this step, we'll work on these.

    .. code-block:: console
          :emphasize-lines: 5,12

          └── sas_robot_driver_myrobot
              ├── CMakeLists.txt
              ├── include
              │   └── sas_robot_driver_myrobot
              │       └── sas_robot_driver_myrobot.hpp
              ├── launch
              │   └── real_robot_launch.py
              ├── package.xml
              ├── scripts
              │   └── joint_interface_example.py
              └── src
                  ├── sas_robot_driver_myrobot.cpp
                  └── sas_robot_driver_myrobot_node.cpp

The files in question are as follows.

.. tab-set::

    .. tab-item:: sas_robot_driver_myrobot.hpp

        :download:`sas_robot_driver_myrobot.hpp <../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/include/sas_robot_driver_myrobot/sas_robot_driver_myrobot.hpp>`

        .. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/include/sas_robot_driver_myrobot/sas_robot_driver_myrobot.hpp
           :language: cpp
           :linenos:
           :lines: 26-

    .. tab-item:: sas_robot_driver_myrobot.cpp

        :download:`sas_robot_driver_myrobot.cpp <../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/src/sas_robot_driver_myrobot.cpp>`

        .. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/src/sas_robot_driver_myrobot.cpp
           :language: cpp
           :linenos:
           :lines: 25-

The example class file has two important design choices to note.

First, we rely on the struct ``RobotDriverMyrobotConfiguration``
to simplify interaction with the constructor. This reduces the amount of code that needs to be changed if a parameter is
added or removed.

.. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/include/sas_robot_driver_myrobot/sas_robot_driver_myrobot.hpp
   :language: cpp
   :linenos:
   :lines: 36-40

Second, we rely on the `PIMPL idiom <https://en.cppreference.com/w/cpp/language/pimpl>`_. This idiom is important to
prevent driver internals to pollute the exported header. This is a very important step to guarantee that your users
don't have to worry about source files specific to the robot and that your package is correctly self-contained. This
is an important design aspect and should not be confused simply with aesthetics or my constant need to sound smart.

.. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/include/sas_robot_driver_myrobot/sas_robot_driver_myrobot.hpp
   :language: cpp
   :linenos:
   :lines: 47-49

When using the `PIMPL idiom <https://en.cppreference.com/w/cpp/language/pimpl>`_ it is important not to forget that the
definition of the implementation class is made in the source. In this example, it is simply a dummy, but in practice
this will depend heavily on the robot drivers.

.. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/src/sas_robot_driver_myrobot.cpp
   :language: cpp
   :linenos:
   :lines: 36-51

Writing the ROS2 Node
---------------------

.. admonition:: In this step, we'll work on this.

    .. code-block:: console
          :emphasize-lines: 13

          └── sas_robot_driver_myrobot
              ├── CMakeLists.txt
              ├── include
              │   └── sas_robot_driver_myrobot
              │       └── sas_robot_driver_myrobot.hpp
              ├── launch
              │   └── real_robot_launch.py
              ├── package.xml
              ├── scripts
              │   └── joint_interface_example.py
              └── src
                  ├── sas_robot_driver_myrobot.cpp
                  └── sas_robot_driver_myrobot_node.cpp

:download:`sas_robot_driver_myrobot_node.cpp <../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/src/sas_robot_driver_myrobot_node.cpp>`

.. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/src/sas_robot_driver_myrobot_node.cpp
   :language: cpp
   :linenos:
   :lines: 25-

Contents of the launch file
---------------------------

.. admonition:: In this step, we'll work on this.

    .. code-block:: console
          :emphasize-lines: 7

          └── sas_robot_driver_myrobot
              ├── CMakeLists.txt
              ├── include
              │   └── sas_robot_driver_myrobot
              │       └── sas_robot_driver_myrobot.hpp
              ├── launch
              │   └── real_robot_launch.py
              ├── package.xml
              ├── scripts
              │   └── joint_interface_example.py
              └── src
                  ├── sas_robot_driver_myrobot.cpp
                  └── sas_robot_driver_myrobot_node.cpp

The launch file will be like so.

:download:`real_robot_launch.py <../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/launch/real_robot_launch.py>`

.. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/launch/real_robot_launch.py
   :language: python
   :linenos:
   :lines: 5-

The parameters should be no surprise defined as follows.

.. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/launch/real_robot_launch.py
   :language: python
   :linenos:
   :lines: 18-23

The most memorable aspect with respect to ``sas`` is that the ``name`` will define the topic prefixes. This is important
to match other elements of ``sas``.

.. literalinclude:: ../../../sas_tutorial_workspace/src/sas_robot_driver_myrobot/launch/real_robot_launch.py
   :language: python
   :linenos:
   :lines: 17

Running the launch file
-----------------------

.. code-block:: console

  ros2 launch sas_robot_driver_myrobot real_robot_launch.py

In another terminal

.. code-block:: console

  ros2 topic list

will show all the available topics that were created for you, freely. Notice that in none of the source files
we created so far had any mention to topics or subscribers. All are created by :program:`sas`.

.. code-block:: console

  /myrobot_1/get/home_states
  /myrobot_1/get/joint_positions_max
  /myrobot_1/get/joint_positions_min
  /myrobot_1/get/joint_states
  /myrobot_1/set/clear_positions
  /myrobot_1/set/homing_signal
  /myrobot_1/set/target_joint_forces
  /myrobot_1/set/target_joint_positions
  /myrobot_1/set/target_joint_velocities
  /parameter_events
  /rosout

Accessing through Python
------------------------

.. admonition:: In this step, we'll work on this.

    .. code-block:: console
          :emphasize-lines: 10

          └── sas_robot_driver_myrobot
              ├── CMakeLists.txt
              ├── include
              │   └── sas_robot_driver_myrobot
              │       └── sas_robot_driver_myrobot.hpp
              ├── launch
              │   └── real_robot_launch.py
              ├── package.xml
              ├── scripts
              │   └── joint_interface_example.py
              └── src
                  ├── sas_robot_driver_myrobot.cpp
                  └── sas_robot_driver_myrobot_node.cpp
