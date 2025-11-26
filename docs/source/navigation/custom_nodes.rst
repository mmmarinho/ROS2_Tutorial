Interface ``nav2`` with custom :program:`ROS2` nodes
====================================================

.. versionadded:: Jazzy

   This section.

.. include:: ../the_topic_is_under_heavy_construction.rst


Objective
---------

We are going to create custom code to interact with the navigation stack, ``nav2``. This will be based on the demo
``tb3_simulation_launch.py``, but it is generally applicable. In the example, ``rviz2`` is used for interactivity.
In this example, we will be able to operate ``nav2`` automatically, without relying on the visualizer.

#. Create a publisher to send the initial pose.
#. Create an action client to send navigation goals.

.. include:: ./tb3_simulation_launch_disclaimer.rst

When dealing with a new stack you might not be familiar with, the first step is to try to make sense of the interfaces
available.

Initial pose topic
++++++++++++++++++

When the ``tb3_simulation_launch.py`` example is running, if we run ``ros2 topic list`` you will see a long
list of topics. Let us filter them out with the keyword *pose*, giving that we're trying to find things related to
pose. We can do so with the command below.

.. code-block:: console

    ros2 topic list | grep pose

The output of that will be the following topics.

.. code-block:: console

    /amcl_pose
    /detected_dock_pose
    /dock_pose
    /filtered_dock_pose
    /goal_pose
    /initialpose
    /staging_pose

This is convenient for us because ``/initialpose`` describes exactly what we want to do in the first step. If you are
annoyed by the fact that the words *initial* and *pose* are not separated by a *_*, so am I.

We can check if this is in fact the correct topic used by :program:`rviz2`. With the demo running, we run, in another
terminal, the following command.

.. code-block:: console

    ros2 topic echo /initialpose

Then, we send the initial position through :program:`rviz2`. We can see that in fact :program:`rviz2` used this topic
to set the initial pose, therefore our guess was correct. Below is not representative output.

.. dropdown:: Output of ros2 topic echo

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

We can understand more about the topic with the following command.

.. code-block:: console

    ros2 topic info /initialpose

This will show us that the topic uses :file:`geometry_msgs/msg/PoseWithCovarianceStamped`. Thence, all we have to do
is create a subscriber with that message type to the topic ``/initialpose``.

.. code-block:: console

    Type: geometry_msgs/msg/PoseWithCovarianceStamped
    Publisher count: 1
    Subscription count: 1

Although it might look somewhat trivial at this stage, there are many :program:`ROS2` concepts at play so that these
connections can be understood.

Navigate to pose action
+++++++++++++++++++++++

When operating the demo through :program:`rviz2`, you might have noticed that sending navigation goals behaves as an
action. It has a goal, feedback, and a result.

With that in mind, we look through the active action, once more filtering for *pose*. We can do so with the following
command.

.. code-block:: console

    ros2 action list | grep pose

This results in the following actions.

.. code-block:: console

    /compute_path_through_poses
    /compute_path_to_pose
    /navigate_through_poses
    /navigate_to_pose


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

.. code-block::

    cd $(ros2 pkg prefix nav2_msgs --share)
    cat action/NavigateToPose.action

.. code-block:: yaml

    #goal definition
    geometry_msgs/PoseStamped pose
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
    builtin_interfaces/Duration navigation_time
    builtin_interfaces/Duration estimated_time_remaining
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

