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

Create the package
------------------

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_gazebo \
    --build-type ament_python \
    --dependencies rclpy ros_gz_interfaces tf2_ros

Files
-----

Sending poses to :program:`Gazebo`
----------------------------------

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

