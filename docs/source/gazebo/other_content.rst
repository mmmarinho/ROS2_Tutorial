
:program:`Gazebo` and :program:`ROS2` structure
-----------------------------------------------

Or as a point cloud

.. tab-set::

    .. tab-item:: Terminal 1: Run the bridge

        .. code-block:: console

            ros2 run ros_gz_bridge parameter_bridge /lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked

    .. tab-item:: Terminal 2: Visualise on rviz2

        .. code-block:: console

            ros2 run rviz2 rviz2 -f camera_with_lidar/link/gpu_lidar

        Then,

            - Add > rviz_default_plugins > LaserScan > OK
            - LaserScan > topic > choose \lidar > ENTER

tf tools such as ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_with_lidar/link/gpu_lidar

A :program:`Gazebo`\-related development in :program:`ROS2` will generally have four packages for each robot or system.
One example is shown below, in the official https://github.com/gazebosim/ros_gz_project_template.

.. code-block::
    :emphasize-lines: 2,5,10,15

    .
    |-- ros_gz_example_application
    |   |-- CMakeLists.txt
    |   `-- package.xml
    |-- ros_gz_example_bringup
    |   |-- CMakeLists.txt
    |   |-- config
    |   |-- launch
    |   `-- package.xml
    |-- ros_gz_example_description
    |   |-- CMakeLists.txt
    |   |-- hooks
    |   |-- models
    |   `-- package.xml
    `-- ros_gz_example_gazebo
        |-- CMakeLists.txt
        |-- README.md
        |-- hooks
        |-- include
        |-- package.xml
        |-- src
        `-- worlds

This means that if you were to develop packages for a robot called ``beautiful_bot`` you'd be expected to create the four packages as follows.

.. code-block::

    .
    |-- beautiful_bot_application
    |-- beautiful_bot_bringup
    |-- beautiful_bot_description
    `-- beautiful_bot_gazebo

According to the developers `here <https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/>`_ and `here <https://github.com/gazebosim/docs/issues/580>`_, our hypotetical packages would have the following roles.

================================= ==================================
:file:`beautiful_bot_application` :program:`ROS2`\-specific code.
:file:`beautiful_bot_bringup`      Project's main launch files and robot configuration.
:file:`beautiful_bot_description` The :file:`.sdf` files for robots and other simulation things.
:file:`beautiful_bot_gazebo`      World :file:`.sdf` file and :Program:`Gazebo` configuration.
================================= ==================================

If you explore the :program:`ROS` packages made avialable by vendors, you will notice this structure or part of it. Some examples

- `Denso <https://github.com/DENSORobot/denso_robot_ros2>`_
- `Kawasaki <https://github.com/Kawasaki-Robotics/khi_robot>`_
- `Franka <https://github.com/frankarobotics/franka_ros2>`_

.. important::

    Owing to the lifecycle of products and resource allocation (we also suffer from this at Universities), you can always expect the official packages for :program:`ROS2` and
    :program:`Gazebo` shipped officially by vendors to considerably lag behind.

:program:`Gazebo` and :program:`ROS2` example
---------------------------------------------

.. note::

   https://github.com/gazebosim/ros_gz_project_template

.. code-block:: console

   ros2 launch ros_gz_example_bringup diff_drive.launch.py
