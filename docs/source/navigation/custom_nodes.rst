Interface ``nav2`` with custom :program:`ROS2` nodes
====================================================

.. versionadded:: Jazzy

   This section.

.. include:: ../the_topic_is_under_heavy_construction.rst


Objective
---------

.. code-block:: console

    ros2 topic list | grep pose

.. code-block:: console

    /amcl_pose
    /detected_dock_pose
    /dock_pose
    /filtered_dock_pose
    /goal_pose
    /initialpose
    /staging_pose

.. code-block:: console

    ros2 action list | grep pose

.. code-block:: console

    /compute_path_through_poses
    /compute_path_to_pose
    /navigate_through_poses
    /navigate_to_pose

Send 2D pose estimate from :program:`rviz2`

.. code-block:: console

    ros2 topic info /initialpose

.. code-block:: console

    Type: geometry_msgs/msg/PoseWithCovarianceStamped
    Publisher count: 1
    Subscription count: 1


.. code-block:: console

    ros2 topic echo /initialpose

.. dropdown::

    .. code-block:: console

        header:
          stamp:
            sec: 247
            nanosec: 791000000
          frame_id: map
        pose:
          pose:
            position:
              x: -2.1046903133392334
              y: -0.3082020580768585
              z: 0.0
            orientation:
              x: 0.0
              y: 0.0
              z: 0.0
              w: 1.0
          covariance:
          - 0.25
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.25
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.06853891909122467
        ---

.. code-block:: console

    ros2 action info /navigate_to_pose

.. code-block:: console

    Action: /navigate_to_pose
    Action clients: 5
        /rviz
        /rviz_navigation_dialog_action_client
        /bt_navigator
        /waypoint_follower
        /docking_server
    Action servers: 1
        /bt_navigator

.. code-block:: console

    ros2 action type /navigate_to_pose

.. code-block:: console

    nav2_msgs/action/NavigateToPose

.. code-block:: console

    ros2 interface show nav2_msgs/action/NavigateToPose

.. code-block:: console

    #goal definition
    geometry_msgs/PoseStamped pose
        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
        Pose pose
            Point position
                float64 x
                float64 y
                float64 z
            Quaternion orientation
                float64 x 0
                float64 y 0
                float64 z 0
                float64 w 1
    string behavior_tree
    ---
    #result definition

    # Error codes
    # Note: The expected priority order of the errors should match the message order
    uint16 NONE=0

    uint16 error_code
    string error_msg
    ---
    #feedback definition
    geometry_msgs/PoseStamped current_pose
        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
        Pose pose
            Point position
                float64 x
                float64 y
                float64 z
            Quaternion orientation
                float64 x 0
                float64 y 0
                float64 z 0
                float64 w 1
    builtin_interfaces/Duration navigation_time
        int32 sec
        uint32 nanosec
    builtin_interfaces/Duration estimated_time_remaining
        int32 sec
        uint32 nanosec
    int16 number_of_recoveries
    float32 distance_remaining

Create the package
------------------

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_nav2 \
    --build-type ament_python \
    --dependencies rclpy geometry_msgs nav2_msgs

.. dropdown::

    .. code-block:: console

        going to create a new package
        package name: python_package_that_uses_nav2
        destination directory: ~/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['rclpy', 'geometry_msgs', 'nav2_msgs']
        creating folder ./python_package_that_uses_nav2
        creating ./python_package_that_uses_nav2/package.xml
        creating source folder
        creating folder ./python_package_that_uses_nav2/python_package_that_uses_nav2
        creating ./python_package_that_uses_nav2/setup.py
        creating ./python_package_that_uses_nav2/setup.cfg
        creating folder ./python_package_that_uses_nav2/resource
        creating ./python_package_that_uses_nav2/resource/python_package_that_uses_nav2
        creating ./python_package_that_uses_nav2/python_package_that_uses_nav2/__init__.py
        creating folder ./python_package_that_uses_nav2/test
        creating ./python_package_that_uses_nav2/test/test_copyright.py
        creating ./python_package_that_uses_nav2/test/test_flake8.py
        creating ./python_package_that_uses_nav2/test/test_pep257.py

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

Files
-----

Sending the initial pose to ``nav2``
------------------------------------

Handling ``nav2`` pose navigation actions
-----------------------------------------

The launch file
+++++++++++++++

Adjusting the :file:`setup.py`
------------------------------

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: ../the_canonical_build_command.rst

Testing
-------

