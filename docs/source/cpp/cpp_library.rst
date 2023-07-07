Creating C++ Libraries (for :program:`ament_cmake`)
===================================================

.. admonition:: The C++ library block for :program:`ament_cmake`

    When adding a new Node in an existing :file:`CMakeLists.txt`, you might benefit from using the following template.

    Remember to

    .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_package_with_a_library/CMakeLists.txt
       :language: cmake
       :lines: 15-63
       :emphasize-lines: 7,12,17,21

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create cpp_package_with_a_library \
    --build-type ament_cmake \
    --dependencies rclcpp


.. code-block:: console

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
