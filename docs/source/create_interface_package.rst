.. versionchanged:: Jazzy

   Added :file:`AmazingQuoteStamped.msg`, :file:`MoveStraightIn2D.action` and simplified service discussion to use :file:`AddPoints.srv`.

Creating a dedicated package for custom interfaces
==================================================

.. warning::

   Despite this push in ROS2 towards having the users define even the simplest of message types, to define new interfaces in ROS2 we must use an :program:`ament_cmake` package. It **cannot** be done with an :program:`ament_python` package.

.. seealso::

   The contents of this session were simplified in this version. A more complex example is shown in https://ros2-tutorial.readthedocs.io/en/humble/service_servers_and_clients.html.

All interfaces in ROS2 must be made in an :program:`ament_cmake` package. We have so far not needed it, but for this scenario we cannot escape. Thankfully, for this we don't need to dig too deep into :program:`CMake`, so fear not.

Overview
--------

We will create a package with the following structure. Besides our good and old :file:`package.xml`, everything else
might be unfamiliar. We will go through those in detail.

.. code:: console

    package_with_interfaces/
    |-- CMakeLists.txt
    |-- action
    |   `-- MoveStraightIn2D.action
    |-- msg
    |   |-- AmazingQuote.msg
    |   `-- AmazingQuoteStamped.msg
    |-- package.xml
    `-- srv
        `-- AddPoints.srv

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
    destination directory: /root/ros2_tutorial_workspace/src
    package format: 3
    version: 0.0.0
    description: TODO: Package description
    maintainer: ['root <murilo.marinho@manchester.ac.uk>']
    licenses: ['TODO: License declaration']
    build type: ament_cmake
    dependencies: ['geometry_msgs']
    creating folder ./package_with_interfaces
    creating ./package_with_interfaces/package.xml
    creating source and include folder
    creating folder ./package_with_interfaces/src
    creating folder ./package_with_interfaces/include/package_with_interfaces
    creating ./package_with_interfaces/CMakeLists.txt

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

The :file:`package.xml` works the same way as when using :file:`ament_python`. However, we no longer have a :file:`setup.py` or :file:`setup.cfg`, everything is handled by the :file:`CMakeLists.txt`.

The :file:`package.xml` dependencies
------------------------------------

Whenever the package has any type of interface, the :file:`package.xml` **must** include three specific dependencies. Namely, the ones highlighted below.
Edit the :file:`package_with_interfaces/package.xml` like so

:download:`~/ros2_tutorial_workspace/src/package_with_interfaces/package.xml <../../ros2_tutorial_workspace/src/package_with_interfaces/package.xml>`

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

:download:`~/ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuote.msg <../../ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuote.msg>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuote.msg
   :language: yaml
   :linenos:

Re-using a message from the same package
++++++++++++++++++++++++++++++++++++++++

With the :file:`AmazingQuote.msg`, we have seen how to use built-in types. Let's use another message, :file:`AmazingQuoteStamped.msg`,  to learn two more possibilities, namely using messages from the same package and messages defined elsewhere.

:download:`~/ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuoteStamped.msg <../../ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuoteStamped.msg>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuoteStamped.msg
   :language: yaml
   :linenos:

Note that if the message is defined in the same package, the package name does not appear in the message (or service) definition. If the message is defined elsewhere, we have to fully specify the package.

In many :program:`ROS2` packages, messages with the suffix ``Stamped`` exist. As a rule, those are the same messages but with a additional
``std_msgs/Header`` so that they can be timestamped.

The service folder
------------------

The convention is to add all services to a folder called :file:`srv`. Let's follow that convention 

.. code:: console

   cd ~/ros2_tutorial_workspace/src/package_with_interfaces
   mkdir srv

.. _The service file:

The service file
----------------

.. note::

   Services cannot be used to define other interfaces.

Add the file :file:`AddPoints.srv` in the :file:`srv` folder with the following contents

:download:`~/ros2_tutorial_workspace/src/package_with_interfaces/srv/AddPoints.srv <../../ros2_tutorial_workspace/src/package_with_interfaces/srv/AddPoints.srv>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/srv/AddPoints.srv
   :language: yaml
   :linenos:

The action folder
-----------------

The convention is to add all actions to a folder called :file:`action`. Let's follow that convention.

.. code:: console

   cd ~/ros2_tutorial_workspace/src/package_with_interfaces
   mkdir action

.. _The action file:

The action file
---------------

.. note::

   Actions cannot be used to define other interfaces.

Add the file :file:`MoveStraightIn2D.action` in the :file:`action` folder with the following contents

:download:`~/ros2_tutorial_workspace/src/package_with_interfaces/action/MoveStraightIn2D.action <../../ros2_tutorial_workspace/src/package_with_interfaces/action/MoveStraightIn2D.action>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/action/MoveStraightIn2D.action
   :language: yaml
   :linenos:


The :file:`CMakeLists.txt` directives
-------------------------------------

.. note:: 

   The order of the :program:`CMake` directives is very important and getting the order wrong can result in bugs with cryptic error messages.

If a package is dedicated to interfaces, there is no need to worry too much about the :program:`CMake` details. We can follow the boilerplate as shown below.
Edit the :file:`package_with_interfaces/CMakeLists.txt` like so

:download:`~/ros2_tutorial_workspace/src/package_with_interfaces/CMakeLists.txt <../../ros2_tutorial_workspace/src/package_with_interfaces/CMakeLists.txt>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/CMakeLists.txt
   :language: cmake
   :linenos:
   :emphasize-lines: 14-40

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
      :lines: 17-28
      :emphasize-lines: 5,8,11

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

    package_with_interfaces/msg/AmazingQuote
    package_with_interfaces/msg/AmazingQuoteStamped
    package_with_interfaces/action/MoveStraightIn2D
    package_with_interfaces/srv/AddPoints
   
and we can further get more specific info on :file:`AmazingQuote` (or :file:`AmazingQuoteStamped`)

.. code:: console

   ros2 interface show package_with_interfaces/msg/AmazingQuote

which returns

.. literalinclude:: ../../ros2_tutorial_workspace/src/package_with_interfaces/msg/AmazingQuote.msg
   :language: yaml

alternatively, we can do the same for :file:`AddPoints`

.. code:: console

   ros2 interface show package_with_interfaces/srv/AddPoints
   
which returns expanded information on each field of the service

.. code:: yaml

    # AddPoints.srv from https://ros2-tutorial.readthedocs.io
    # Adds the values of points `a` and `b` to give the output `result`
    geometry_msgs/Point a
            float64 x
            float64 y
            float64 z
    geometry_msgs/Point b
            float64 x
            float64 y
            float64 z
    ---
    geometry_msgs/Point result
            float64 x
            float64 y
            float64 z


Lastly, we can do the same for :file:`MoveStraightIn2D`

.. code:: console

   ros2 interface show package_with_interfaces/action/MoveStraightIn2D

which returns expanded information about all fields of the action

.. code:: yaml

    # MoveStraightIn2D.action from https://ros2-tutorial.readthedocs.io
    # Attempts to move from initial position to `desired_position`.
    # Returns `final_position` achieved.
    # Feedback is the norm of the error between `initial_position` and the current position.
    geometry_msgs/Point desired_position
            float64 x
            float64 y
            float64 z
    ---
    geometry_msgs/Point final_position
            float64 x
            float64 y
            float64 z
    ---
    float32 error_norm