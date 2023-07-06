Creating C++ Nodes (for :program:`ament_cmake`)
===============================================

.. admonition:: The C++ binary block for :program:`ament_cmake`

    When adding a new Node in an existing :file:`CMakeLists.txt`, you might benefit from using the following template.
    There are many things to add, but remember that the name of the binary is repeated in 5 different places, as highlighted below.


    .. code-block:: CMake
        :emphasize-lines: 5,10,14,18,20

        ######################
        # CPP Binary [BEGIN] #
        # vvvvvvvvvvvvvvvvvv #

        add_executable(sample_cpp_node 
            src/sample_cpp_node.cpp
                       
        )

        target_link_libraries(sample_cpp_node

        )

        target_include_directories(sample_cpp_node PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)

        target_compile_features(sample_cpp_node PUBLIC c_std_99 cxx_std_17)  
        
        install(TARGETS sample_cpp_node
          DESTINATION lib/${PROJECT_NAME})

        # ^^^^^^^^^^^^^^^^ #
        # CPP BINARY [END] #
        ####################

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

The :file:`.hpp`
^^^^^^^^^^^^^^^^

:download:`print_forever_cpp_node.hpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_cpp_node.hpp>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_cpp_node.hpp
   :language: cpp
   :linenos:


The :file:`.cpp`
^^^^^^^^^^^^^^^^

:download:`print_forever_cpp_node.cpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_cpp_node.cpp>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_node/src/print_forever_cpp_node.cpp
   :language: cpp
   :linenos:

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

   ros2 run cpp_package_with_a_node sample_cpp_node

.. code-block:: console

    [INFO] [1688620414.406930812] [print_forever_cpp]: Printed 0 times.
    [INFO] [1688620414.906890884] [print_forever_cpp]: Printed 1 times.
    [INFO] [1688620415.406907619] [print_forever_cpp]: Printed 2 times.
    [INFO] [1688620415.906881003] [print_forever_cpp]: Printed 3 times.
    [INFO] [1688620416.406900108] [print_forever_cpp]: Printed 4 times.
    [INFO] [1688620416.906886691] [print_forever_cpp]: Printed 5 times.
    [INFO] [1688620417.406881803] [print_forever_cpp]: Printed 6 times.
    [INFO] [1688620417.906858551] [print_forever_cpp]: Printed 7 times.
    [INFO] [1688620418.406894922] [print_forever_cpp]: Printed 8 times.

:kbd:`CTRL+C`

.. code-block:: console

    [INFO] [1688620418.725674401] [rclcpp]: signal_handler(signum=2)
