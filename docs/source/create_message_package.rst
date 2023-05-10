Creating a dedicated package for interfaces
===========================================

.. warning::

   Despite this push in ROS2 towards having the users define even the simplest of message types, to define new interfaces in ROS2 we must use an :program:`ament_cmake` package. It **cannot** be done with an :program:`ament_python` package.

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

The :file:`package.xml` dependencies
------------------------------------

Whenever the package has any type of interface, the :file:`package.xml` **must** include three specific dependencies. Namely, the ones highlighted below.
Edit the :file:`package_with_interfaces/package.xml` like so

:download:`package.xml <../../ros2_tutorial_workspace/src/package_with_interfaces/package.xml>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/package.xml
   :language: xml
   :linenos:
   :emphasize-lines: 12,13,14

The message folder
------------------

The convention is to add all messages to a folder called :file:`msg`. Let's follow that convention 

.. code:: console

   cd ~/ros2_tutorial_workspace/src
   mkdir msg

The message file
----------------

.. note::

   Here is a list of available `built-in types <https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html#field-types>`_ for ROS2 interfaces.

Let us create a message file to transfer inspirational quotes between Nodes. For example, the one below.

.. epigraph::

   Use the force, Pikachu!

   -- Uncle Ben

There are many ways to represent this, but for the sake of the example let us give each message an :code:`id` and two rather obvious fields.
Create a file called :file:`AmazingQuote.msg` in the folder :file:`msg` that we just created with the following contents.

:download:`AmazingQuote.msg <../../ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuote.msg>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuote.msg
   :language: yaml
   :linenos:

The :file:`CMakeLists.txt` directives
-------------------------------------

.. note:: 

   The order of the :program:`CMake` directives is very important. In addition, building several nodes, libraries, and messages in the same project can be complex and generate a lot of issues given the interactions between the targets and their dependencies.

If a package is dedicated for interfaces, there is no need to worry too much about the :program:`CMake` details. We can follow the boilerplate as shown below.
Edit the :file:`package_with_interfaces/CMakeLists.txt` like so

:download:`CMakeLists.txt <../../ros2_tutorial_workspace/src/package_with_interfaces/CMakeLists.txt>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/CMakeLists.txt
   :language: cmake
   :linenos:
   :emphasize-lines: 13-25

If additional interfaces are required, the only point of change is this. We can add one per line, keeping the identation.

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/CMakeLists.txt
   :language: cmake
   :lines: 16-18
   
