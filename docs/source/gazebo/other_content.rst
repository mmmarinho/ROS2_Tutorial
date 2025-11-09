
:program:`Gazebo` and :program:`ROS2` structure
-----------------------------------------------

These are the internal :program:`Gazebo` topics created in this scene for each sensor. Here are some pairings with
example launch files, although the mapping is not always one-to-one.

===============  ================ ===============================================================================================================================
Sensor type      Topic name       Demo
===============  ================ ===============================================================================================================================
camera           camera_alone     `image_bridge.launch.py <https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim_demos/launch/image_bridge.launch.py>`_
depth_camera     depth_camera     `depth_camera.launch.py <https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim_demos/launch/depth_camera.launch.py>`_
gpu_lidar        lidar            `gpu_lidar_bridge.launch.py <https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim_demos/launch/gpu_lidar_bridge.launch.py>`_
thermal_camera   thermal_camera   **N/A**
rgbd_camera      rgbd_camera      `rgbd_camera_bridge.launch.py <https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim_demos/launch/rgbd_camera_bridge.launch.py>`_
===============  ================ ===============================================================================================================================


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


Entities
++++++++

gz service -i --service /world/shapes/set_pose/blocking

Service providers [Address, Request Message Type, Response Message Type]:
  tcp://172.16.191.128:36717, gz.msgs.Pose, gz.msgs.Boolean

ros2 run ros_gz_bridge \
parameter_bridge \
/world/shapes/set_pose/blocking@geometry_msgs/msg/PoseStamped]gz.msgs.Pose

Getting entity pose information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/pose_publisher.sdf

Add a ``Pose Publisher`` to the entity and don't define a name. You can find the new topic by looking through the
output of :program:`gz topic list` and you will find the following topic.

    /world/shapes/model/sphere/pose

Then, we check the :program:`Gazebo` message type with the following command.

.. code-block::

    gz topic -i --topic /world/shapes/model/sphere/pose

The result of the command will be the following.

.. code-block::

    Publishers [Address, Message Type]:
      tcp://172.16.191.128:33987, gz.msgs.Pose
    No subscribers on topic [/world/shapes/model/sphere/pose]

Because this will be an unilateral bridge from :program:`Gazebo` to :program:`ROS2`, we use the ``[`` instead of ``@``
as follows.

.. code-block::

    ros2 run ros_gz_bridge \
    parameter_bridge \
    /world/shapes/model/sphere/pose@geometry_msgs/msg/Pose[gz.msgs.Pose

We can see the contents of the topic with the following command.

.. code-block::

    ros2 topic echo /world/shapes/model/sphere/pose

Which will result in the following output, showing the pose of the ``sphere`` entity.

.. code-block::

    ---
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

.. admonition:: Exercises

    Can you do the same for the other shapes available in the scene? For example, for


Setting the pose
----------------

.. code-block:: console

    gz service -s /world/shapes_with_pose_publisher/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req 'data: "Hello"'


https://docs.ros.org/en/rolling/p/ros_gz_sim_demos/

run ros_gz_bridge parameter_bridge /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose



ros2 run ros_gz_sim set_entity_pose [--name NAME | --id ID] [--type TYPE] [--pos X Y Z] [--quat X Y Z W | --euler ROLL PITCH YAW]

