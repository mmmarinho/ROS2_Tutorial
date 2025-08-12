Creating C++ Nodes (for :program:`ament_cmake`)
===============================================

.. admonition:: The C++ binary block for :program:`ament_cmake`

    .. dropdown:: TL;DR

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

which outputs

.. dropdown:: ros2 pkg create output

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

Package-related sources
-----------------------

.. admonition:: In this step, we'll work on these.
    
    .. code-block:: console
        :emphasize-lines: 2,6
    
        cpp_package_with_a_node
        ├── CMakeLists.txt
        ├── include
        │   └── cpp_package_with_a_node
        │       └── .placeholder
        ├── package.xml
        └── src
            ├── print_forever_node.cpp
            ├── print_forever_node.hpp
            └── print_forever_node_main.cpp

The files already exist, we just need to modify them as follows

.. tab-set::

    .. tab-item:: package.xml

        The :file:`package.xml` works the same way as in :program:`ament_python`, with the exception of the two lines about :program:`ament_cmake` shown below.

        :download:`package.xml <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/package.xml>`

        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/package.xml
           :language: xml
           :linenos:
           :emphasize-lines: 10,18

    .. tab-item:: CMakeLists.txt

        A *one-size-fits-most* solution is shown below. For each new Node we add a block to the :file:`CMakeLists.txt` with the following format.

        :download:`CMakeLists.txt <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/CMakeLists.txt>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/CMakeLists.txt
           :language: cmake
           :linenos:
           :emphasize-lines: 12-47

Making C++ ROS2 Nodes
---------------------

.. admonition:: (Murilo's) ``rclcpp`` best practices 

   For each new C++ Node, we make three files following the style below.

   For a Node called ``print_forever_node`` we have

   #. :file:`src/print_forever_node.hpp` with the Node's class definition. In general, this is not exported to other packages, so it should not be in the package's :file:`include` folder.
   #. :file:`src/print_forever_node.cpp` with the Node's class implementation.
   #. :file:`src/print_forever_node_main.cpp` with the Node's main function implementation.

.. admonition:: In this step, we'll work on these.

    .. code-block:: console
        :emphasize-lines: 7-10
    
        cpp_package_with_a_node
        ├── CMakeLists.txt
        ├── include
        │   └── cpp_package_with_a_node
        │       └── .placeholder
        ├── package.xml
        └── src
            ├── print_forever_node.cpp
            ├── print_forever_node.hpp
            └── print_forever_node_main.cpp

These files do not exist, so we'll create them.

.. tab-set::

    .. tab-item:: folder

       Create the folder.

       .. code-block:: console

          cd ~/ros2_tutorial_workspace/src/cpp_package_with_a_node
          mkdir src

    .. tab-item:: src/..._node.hpp

        Similar to what we did in Python, we inherit from ``rclcpp::Node``. Whatever is different is owing to differences in languages.
  
        :download:`print_forever_node.hpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node.hpp>`

        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node.hpp
           :language: cpp
           :linenos:
           :lines: 24-
           :emphasize-lines: 9

    .. tab-item:: src/..._node.cpp

        The implementation has nothing special, just don't forget to initialize the parent class, ``rclcpp::Node``, with the name of the node. 

        :download:`print_forever_node.cpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node.cpp>`

        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node.cpp
           :language: cpp
           :linenos:
           :lines: 24-
           :emphasize-lines: 7

    .. tab-item::  src/..._main.cpp 

        Given that we are using ``rclcpp::spin()``, there is nothing special here either. Just remember to not mess up the ``std::make_shared`` and always use perfect forwarding.
        The ``rclcpp::spin()`` handles the ``SIGINT`` when we, for example, press :kbd:`CTRL+C` on the terminal. It is not perfect, but it does the trick for simple nodes like this one.
        
        :download:`print_forever_node_main.cpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node_main.cpp>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_node_main.cpp
           :language: cpp
           :linenos:
           :lines: 24-
           :emphasize-lines: 11


Add a :file:`.placeholder` if your :file:`include/<PACKAGE_NAME>` is empty
--------------------------------------------------------------------------

.. warning::

   If you don't do this and add this package as a git repository without any files on the :file:`include/`, :program:`CMake` might return with an error when trying to compile your package.

.. code-block:: console
    :emphasize-lines: 5

    cpp_package_with_a_node
    ├── CMakeLists.txt
    ├── include
    │   └── cpp_package_with_a_node
    │       └── .placeholder
    ├── package.xml
    └── src
        ├── print_forever_node.cpp
        ├── print_forever_node.hpp
        └── print_forever_node_main.cpp

Empty directories will `not be tracked by git <https://stackoverflow.com/questions/115983/how-do-i-add-an-empty-directory-to-a-git-repository>`_. A file has to be added to the index. We can create an empty file in the :file:`include` folder as follows

.. code-block:: console

   cd ~/ros2_tutorial_workspace/src/cpp_package_with_a_node/src
   touch include/cpp_package_with_a_node/.placeholder

Running a C++ Node
------------------

As simple as it has always been, see :ref:`Running a node`.

.. code-block:: console

   ros2 run cpp_package_with_a_node print_forever_node

which returns

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

and we'll use :kbd:`CTRL+C` to stop the node, resulting in

.. code-block:: console

    [INFO] [1688620418.725674401] [rclcpp]: signal_handler(signum=2)
