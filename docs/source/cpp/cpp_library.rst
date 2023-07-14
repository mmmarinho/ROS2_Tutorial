Creating C++ Libraries (for :program:`ament_cmake`)
===================================================

.. admonition:: The C++ library block for :program:`ament_cmake`

    .. dropdown:: TL;DR
    
        When your project exports a library, you might benefit from using the following template.
        Note that there is, in general, no reason to define multiple libraries. A single shared library can hold all the
        content that you want to export from a package, hence the library named ``${PROJECT_NAME}``.
    
        Remember to
    
        #. Add all exported headers to :file:`include/<PACKAGE_NAME>` otherwise other packages cannot see it.
        #. Add all source files of the library to ``add_library``.
        #. Add all ROS2 dependencies of the library to ``ament_target_dependencies``.
        #. Add **ALL** dependencies for which you used ``find_package`` to ``ament_export_dependencies``, otherwise dependencies might become complex for projects that use your library.
        #. Add any other (**NOT ROS2**) libraries to ``target_link_libraries``.
    
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/CMakeLists.txt
           :language: cmake
           :lines: 14-63
           :emphasize-lines: 9,14,27,32

The base package can be created with

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create cpp_package_with_a_library \
    --build-type ament_cmake \
    --dependencies rclcpp

resulting in the following output

.. dropdown:: ros2 pkg create output

    .. code-block:: console
        :emphasize-lines: 16,17,18
    
        ros2 pkg create cpp_package_with_a_library \
        --build-type ament_cmake \
        --dependencies rclcpp
        going to create a new package
        package name: cpp_package_with_a_library
        destination directory: /home/murilo/ROS2_Tutorial/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['murilo <murilomarinho@ieee.org>']
        licenses: ['TODO: License declaration']
        build type: ament_cmake
        dependencies: ['rclcpp']
        creating folder ./cpp_package_with_a_library
        creating ./cpp_package_with_a_library/package.xml
        creating source and include folder
        creating folder ./cpp_package_with_a_library/src
        creating folder ./cpp_package_with_a_library/include/cpp_package_with_a_library
        creating ./cpp_package_with_a_library/CMakeLists.txt
        
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
    
        cpp_package_with_a_library
        ├── CMakeLists.txt
        ├── include
        │   └── cpp_package_with_a_library
        │       └── sample_class.hpp
        ├── package.xml
        └── src
            ├── sample_class.cpp
            ├── sample_class_local_node.cpp
            ├── sample_class_local_node.hpp
            └── sample_class_local_node_main.cpp

The files already exist, we just need to modify them as follows

.. tab-set::

    .. tab-item:: package.xml

        Nothing new here.

        :download:`package.xml <../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/package.xml>`

        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/package.xml
           :language: xml
           :linenos: 

    .. tab-item:: CMakeLists.txt

        A *one-size-fits-most* solution is shown below. We don't need to add multiple libraries, so a single library can hold all the content you might want to export. The user of the library will see it nicely split by your header files, so it will be as neat as you make them.

        Note that, because the local Node depends on the library being exported by this project, it needs to explicitly link to it.

        :download:`CMakeLists.txt <../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/CMakeLists.txt>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/CMakeLists.txt
           :language: cmake
           :linenos:
           :emphasize-lines: 14-46,86


Library sources
---------------

.. admonition:: In this step, we'll work on these.
    
    .. code-block:: console
        :emphasize-lines: 5,8
    
        cpp_package_with_a_library
        ├── CMakeLists.txt
        ├── include
        │   └── cpp_package_with_a_library
        │       └── sample_class.hpp
        ├── package.xml
        └── src
            ├── sample_class.cpp
            ├── sample_class_local_node.cpp
            ├── sample_class_local_node.hpp
            └── sample_class_local_node_main.cpp

.. tab-set::

    .. tab-item:: sample_class.hpp  

        A class that does a bunch of nothing, but that depends on Eigen3 and Qt, as an example.

        :download:`sample_class.hpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/include/cpp_package_with_a_library/sample_class.hpp>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/include/cpp_package_with_a_library/sample_class.hpp
           :language: cpp
           :linenos:
           :lines: 24-

    .. tab-item:: sample_class.cpp

        :download:`sample_class.cpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/src/sample_class.cpp>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/src/sample_class.cpp
           :language: cpp
           :linenos:
           :lines: 24-


Sources for a local node that uses the library
----------------------------------------------

.. admonition:: In this step, we'll work on these.
    
    .. code-block:: console
        :emphasize-lines: 9-11
    
        cpp_package_with_a_library
        ├── CMakeLists.txt
        ├── include
        │   └── cpp_package_with_a_library
        │       └── sample_class.hpp
        ├── package.xml
        └── src
            ├── sample_class.cpp
            ├── sample_class_local_node.cpp
            ├── sample_class_local_node.hpp
            └── sample_class_local_node_main.cpp

Just in case you need to have a node, in the same package, that also uses the library exported by this package. Nothing too far from what we have already done.

.. tab-set::

    .. tab-item:: sample_class_local_node.cpp

        :download:`sample_class.cpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/src/sample_class_local_node.cpp>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/src/sample_class_local_node.cpp
           :language: cpp
           :linenos:
           :lines: 24-

    .. tab-item:: sample_class_local_node.hpp

        :download:`sample_class_local_node.cpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/src/sample_class_local_node.hpp>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/src/sample_class_local_node.hpp
           :language: cpp
           :linenos:
           :lines: 24-

    .. tab-item:: sample_class_local_node_main.cpp

        :download:`sample_class.cpp <../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/src/sample_class_local_node_main.cpp>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/src/sample_class_local_node_main.cpp
           :language: cpp
           :linenos:
           :lines: 24-

