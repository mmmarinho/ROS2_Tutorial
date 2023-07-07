Creating C++ Nodes (for :program:`ament_cmake`)
===============================================

.. admonition:: The C++ binary block for :program:`ament_cmake`

    When adding a new Node in an existing :file:`CMakeLists.txt`, you might benefit from using the following template.

    Remember to:

    #. Add **ALL** dependencies (including ROS2 ones) with ``find_package``, if applicable.

        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/CMakeLists.txt
           :language: cmake
           :lines: 8-10

    #. Change ``print_forever_node`` to the name of your Node.
    #. Add all source files to ``add_executable``.
    #. Add all ROS2 dependencies of this binary to ``ament_target_dependencies``.
    #. Add any other (**NOT ROS2**) libraries to ``target_link_libraries``.

    .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/CMakeLists.txt
       :language: cmake
       :lines: 12-47
       :emphasize-lines: 7,12,17,21
 

Create the package
------------------

.. warning::

   We'll skip using the ``--node-name`` option to create the Node template, because, currently, it generates a Node and a :file:`CMakeLists.txt` different from my advice.

.. code-block:: console

  cd ~/ros2_tutorial_workspace/src
  ros2 pkg create cpp_package_with_a_node \
  --build-type ament_cmake \
  --dependencies rclcpp

.. code-block:: console
  :emphasize-lines: 13-16

        going to create a new package
        package name: cpp_package_with_a_node
        destination directory: /home/murilo/ROS2_Tutorial/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['murilo <murilomarinho@ieee.org>']
        licenses: ['TODO: License declaration']
        build type: ament_cmake
        dependencies: ['rclcpp']
        creating folder ./cpp_package_with_a_node
        creating ./cpp_package_with_a_node/package.xml
        creating source and include folder
        creating folder ./cpp_package_with_a_node/src
        creating folder ./cpp_package_with_a_node/include/cpp_package_with_a_node
        creating ./cpp_package_with_a_node/CMakeLists.txt
        
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

The :file:`package.xml`
--------------------------

:download:`package.xml <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/package.xml>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/package.xml
   :language: xml
   :linenos:
   :emphasize-lines: 10

The :file:`CMakeLists.txt`
--------------------------

:download:`CMakeLists.txt <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/CMakeLists.txt>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/CMakeLists.txt
   :language: cmake
   :linenos:

The ``PrintForeverCPP`` class
-----------------------------

.. admonition:: (Murilo's) ``rclcpp`` best practices 

   Anything that is a subclass of ``rclcpp::Node`` will have the suffix :file:`_node`.

The :file:`.hpp`
^^^^^^^^^^^^^^^^

:download:`print_forever.hpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node.hpp>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node.hpp
   :language: cpp
   :linenos:
   :lines: 24-


The :file:`.cpp`
^^^^^^^^^^^^^^^^

:download:`print_forever.cpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node.cpp>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node.cpp
   :language: cpp
   :linenos:
   :lines: 24-

The ``main()`` function
-----------------------

.. admonition:: (Murilo's) ``rclcpp`` best practices 

   Anything contains the ``main()`` function of a node will have the suffix :file:`_node_main.cpp`.

:download:`print_forever_cpp_node.cpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node_main.cpp>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node_main.cpp
   :language: cpp
   :linenos:
   :lines: 24-

Add a :file:`.placeholder` if your :file:`include/<PACKAGE_NAME>` is empty
--------------------------------------------------------------------------

.. warning::

   If you don't do this and add this package as a git repository without any files on the :file:`include/`, :program:`CMake` might return with an error when trying to compile your package.

Empty directories will `not be tracked by git <https://stackoverflow.com/questions/115983/how-do-i-add-an-empty-directory-to-a-git-repository>`_. A file has to be added to the index. We can create an empty file in the :file:`include` folder as follows

.. code-block:: console

   cd ~/ros2_tutorial_workspace/src/cpp_package_with_a_node/src
   touch include/cpp_package_with_a_node/.placeholder

Running C++ Node
----------------

.. code-block:: console

   ros2 run cpp_package_with_a_node print_forever_node

.. code-block:: console

    [INFO] [1688620414.406930812] [print_forever_node]: Printed 0 times.
    [INFO] [1688620414.906890884] [print_forever_node]: Printed 1 times.
    [INFO] [1688620415.406907619] [print_forever_node]: Printed 2 times.
    [INFO] [1688620415.906881003] [print_forever_node]: Printed 3 times.
    [INFO] [1688620416.406900108] [print_forever_node]: Printed 4 times.
    [INFO] [1688620416.906886691] [print_forever_node]: Printed 5 times.
    [INFO] [1688620417.406881803] [print_forever_node]: Printed 6 times.
    [INFO] [1688620417.906858551] [print_forever_node]: Printed 7 times.
    [INFO] [1688620418.406894922] [print_forever_node]: Printed 8 times.

:kbd:`CTRL+C`

.. code-block:: console

    [INFO] [1688620418.725674401] [rclcpp]: signal_handler(signum=2)
