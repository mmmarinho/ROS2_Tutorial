Creating a dedicated package for custom interfaces
==================================================

.. warning::

   Despite this push in ROS2 towards having the users define even the simplest of message types, to define new interfaces in ROS2 we must use an :program:`ament_cmake` package. It **cannot** be done with an :program:`ament_python` package.

All interfaces in ROS2 must be made in an :program:`ament_cmake` package. We have so far not needed it, but for this scenario we cannot escape. Thankfully, for this we don't need to dig too deep into :program:`CMake`, so fear not.

Creating the package
--------------------

There isn't a template for message-only packages using :program:`ros2 pkg create`. We'll need to build on top of a mostly empty :program:`ament_cmake` package.

To take this chance to also learn how to nest messages on other interfaces, we also add the dependency on :code:`geometry_msgs`.

.. code:: console

  cd ~/ros2_tutorial_workspace/src
  ros2 pkg create package_with_interfaces \
  --build-type ament_cmake \
  --dependencies geometry_msgs

which again shows our beloved wall of text, with a few highlighted differences because of it being a :program:`ament_cmake` package.

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
    dependencies: [geometry_msgs]
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
   :emphasize-lines: 14-16

The message folder
------------------

The convention is to add all messages to a folder called :file:`msg`. Let's follow that convention 

.. code:: console

   cd ~/ros2_tutorial_workspace/src/package_with_interfaces
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

The service folder
------------------

The convention is to add all services to a folder called :file:`srv`. Let's follow that convention 

.. code:: console

   cd ~/ros2_tutorial_workspace/src/package_with_interfaces
   mkdir srv

.. _The service file:

The service file
----------------

With the :file:`AmazingQuote.msg`, we have seen how to use built-in types. Let's use the service to learn two more possibilities. Let us use a message from the same package and a message from another package. Services cannot be used to define other services.

Add the file :file:`WhatIsThePoint.srv` in the :file:`srv` folder with the following contents

:download:`WhatIsThePoint.srv <../../ros2_tutorial_workspace/src/package_with_interfaces/srv/WhatIsThePoint.srv>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/srv/WhatIsThePoint.srv
   :language: yaml
   :linenos:

Note that if the message is defined in the same package, the package name does not appear in the service or message definition. If the message is defined elsewhere, we have to specify it.

The :file:`CMakeLists.txt` directives
-------------------------------------

.. note:: 

   The order of the :program:`CMake` directives is very important and getting the order wrong can result in bugs with cryptic error messages.

If a package is dedicated to interfaces, there is no need to worry too much about the :program:`CMake` details. We can follow the boilerplate as shown below.
Edit the :file:`package_with_interfaces/CMakeLists.txt` like so

:download:`CMakeLists.txt <../../ros2_tutorial_workspace/src/package_with_interfaces/CMakeLists.txt>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/CMakeLists.txt
   :language: cmake
   :linenos:
   :emphasize-lines: 14-36

What to do when adding new interfaces?
--------------------------------------

.. admonition:: **TL;DR** Adding new interfaces

         #. Add new dependencies to :file:`package.xml`
         #. Add each new interface file to :code:`set(interface_files ...)`
         #. Add new dependencies to :code:`rosidl_generate_interfaces(... DEPENDENCIES ...)`

         Yes, you **MUST** add the same dependency in two places!

If additional interfaces are required

#. Modify the :file:`package.xml` to have any additional dependencies. See :ref:`Handling dependencies` for more details.

#. Add each new interface file to :code:`set(interface_files ...)`

   .. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/CMakeLists.txt
      :language: cmake
      :lines: 17-24
      :emphasize-lines: 4, 7

.. note::

   There are ways to use :program:`CMake` directives to automatically add all files in a given folder and provide other conveniences. In hindsight, that might seem to reduce our burden. However, the method described herein is the one used in the official ROS2 packages (e.g. `geometry_msgs <https://github.com/ros2/common_interfaces/blob/f4eac72f0bbd70f7955a5f709d4a6705eb6ca7e8/geometry_msgs/CMakeLists.txt>`_), so let us trust that they have good reasons for it.

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: the_canonical_build_command.rst

Getting info on custom interfaces
---------------------------------

As long as the package has been correctly built and sourced, we can easily get information on its interfaces using :program:`ros2 interface`.

For instance, running

.. code:: console

   ros2 interface package package_with_interfaces
   
returns

.. code:: console

   package_with_interfaces/srv/WhatIsThePoint
   package_with_interfaces/msg/AmazingQuote
   
and we can further get more specific info on :file:`AmazingQuote.msg`

.. code:: console

   ros2 interface show package_with_interfaces/msg/AmazingQuote

which returns

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuote.msg
   :language: yaml

alternatively, we can do the same for :file:`WhatIsThePoint.srv`

.. code:: console

   ros2 interface show package_with_interfaces/srv/WhatIsThePoint
   
which returns expanded information on each field of the service


.. code:: yaml

   # WhatIsThePoint.srv from https://ros2-tutorial.readthedocs.io
   # Receives an AmazingQuote and returns what is the point
   AmazingQuote quote
      int32 id
      string quote
      string philosopher_name
   ---
   geometry_msgs/Point point
      float64 x
      float64 y
      float64 z

