Using :program:`Gazebo`
=======================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

.. hint::

    You can start the simulator with

    .. code-block:: console

        gz sim

In this section we will see the basic operations related to :program:`Gazebo` and create our own :file:`sdf` file.

Basic functionality
-------------------

.. hint::

    In the ``Understanding the GUI`` tutorial, you can skip the part about using ``--force-version``.
    The version changes quite frequently. It could be safely changed to.

    .. code-block:: console

        gz sim --force-version 8 shapes.sdf


    But you probably should not have multiple versions of :program:`Gazebo` installed anyway!

These two official tutorials cover the basic functionality of :program:`Gazebo`.
They are very well made with up-to-date images and videos. Please go through them to familiarise yourself with the basic functionality.

- `Understanding the GUI <https://gazebosim.org/docs/harmonic/gui/>`_.
- `Manipulating Models <https://gazebosim.org/docs/harmonic/manipulating_models/>`_.

Fun with plugins
----------------

Check the guides below for basic functionality.

- `Apply force torque plugin <https://gazebosim.org/api/sim/8/apply_force_torque.html>`_.
- `Mouse drag plugin <https://gazebosim.org/api/sim/8/mouse_drag.html>`_.

World files :file:`.sdf`
------------------------

.. seealso::

    Official documentation: https://gazebosim.org/docs/harmonic/sdf_worlds/

Let us start with the sample scene  :file:`shapes.sdf`. This example is `available in the official repository
<https://raw.githubusercontent.com/gazebosim/gz-sim/refs/heads/gz-sim8/examples/worlds/shapes.sdf>`_. It is shown
below to save you a click. The format is :file:`xml`, which should be a familiar file format by now.

.. dropdown:: Contents of :file:`shapes.sdf`

    .. rli:: https://raw.githubusercontent.com/gazebosim/gz-sim/refs/heads/gz-sim8/examples/worlds/shapes.sdf
        :language: xml

As you could have noticed, even simple world files can have a relatively complex format. The example :file:`shapes.sdf`, that has only a few simple
objects, is difficult to parse visually. For most things, it is easier to use :program:`Gazebo` itself and limit
and direct changes in :file:`.sdf` files.

Adding or removing things directly on :program:`Gazebo` is sometimes not repeatable with possible GUI changes across versions.
In addition, it is not as natural for a text-based tutorial such as this one. In those scenarios, I will show how to edit the file directly.

This tutorial will limit itself on using default :file:`.sdf` files and making small modifications. Our interest is to be
able obtain and send useful information from :program:`Gazebo` to :program:`ROS`.

You will notice in this file some of the most common elements of :file:`.sdf`. These are not the only elements and this
list is not meant to be comprehensive.

- ``<sdf>`` is the highest-level element, defining the entire :file:`.sdf`.
- ``<world name="world_name">`` encloses the elements in this world and gives it a name. This name, rather than the name
  of the file, will be used when exposing topics and services.
- ``<scene>`` has broad elements of the scene, such as background color.
- Entities can include:
    - ``<light>`` a source of light
    - ``<model>`` a model in the simulation, such as simple shapes or a robot, that can include other information such as
      ``<link>``, ``<visual>``, and ``<collision>``.

.. note::

    If your application demands it, you will need specialised programs to create 3D models, such as `Blender <https://www.blender.org/>`_.
    It won't be usually manageable to create complicated models directly on :program:`Gazebo`.

In this example, because we're using simple shapes supported by :program:`Gazebo`, they are defined with ``<geometry>``
elements.

Gazebo topics and services
--------------------------

.. caution::

    Internal :program:`Gazebo` topics and services are **not** the same as :program:`ROS2` topics and services.

A :program:`Gazebo` scene will have a number of internal topics and services. We can interact with them using :program:`gz`
commands. The messages that flow through :program:`Gazebo` are based on `protobuf <https://protobuf.dev>`_, not
:program:`ROS2` IDL.

Let us use an example with sensors. This example is `available in the official repository
<https://raw.githubusercontent.com/gazebosim/gz-sim/refs/heads/gz-sim8/examples/worlds/sensors_demo.sdf>`_.

.. dropdown:: Contents of :file:`sensors_demo.sdf`

    .. rli:: https://raw.githubusercontent.com/gazebosim/gz-sim/refs/heads/gz-sim8/examples/worlds/sensors_demo.sdf
        :language: xml
        :linenos:

To enable physical simulation, including sensor, we can see that a new tag, ``<plugins>`` is added. These are paramount
to guarantee that sensors will work. Further, they create important topics and services in :program:`Gazebo`. Here are
the plugins active on this file.

.. rli:: https://raw.githubusercontent.com/gazebosim/gz-sim/refs/heads/gz-sim8/examples/worlds/sensors_demo.sdf
    :language: xml
    :lines: 7-27

We will now start to interact with :program:`Gazebo`. For this section, suppose that we have the scene always open with the following command.

.. tip::

    You will see online, very frequently, the command using ``-v4``. For instance, as follows. This allows :program:`Gazebo`
    to output many messages to the terminal in which it was opened to help you understand what is happening.

    .. code-block::

        gz sim -v4 sensors_demo.sdf

.. code-block:: console

    gz sim sensors_demo.sdf

A frequently used tool to allow us to inspect internal Gazebo topics will be, in another terminal, to run the following
command. The ``-l`` flag allows us to list :program:`Gazebo` topics.

.. caution::

    :program:`Gazebo` topics and services are **not** the same as :program:`ROS2` topics and services.

.. code-block:: console

    gz topic -l

If the correct scene is running on :program:`Gazebo`, the output should be similar to the following output.

.. code-block:: console

    /camera
    /camera_alone
    /camera_info
    /clock
    /depth_camera
    /depth_camera/performance_metrics
    /depth_camera/points
    /gazebo/resource_paths
    /gui/camera/pose
    /gui/currently_tracked
    /gui/track
    /lidar
    /lidar/points
    /rgbd_camera/camera_info
    /rgbd_camera/depth_image
    /rgbd_camera/image
    /rgbd_camera/performance_metrics
    /rgbd_camera/points
    /sensors/marker
    /stats
    /thermal_camera
    /world/lidar_sensor/clock
    /world/lidar_sensor/dynamic_pose/info
    /world/lidar_sensor/pose/info
    /world/lidar_sensor/scene/deletion
    /world/lidar_sensor/scene/info
    /world/lidar_sensor/state
    /world/lidar_sensor/stats
    /world/lidar_sensor/light_config
    /world/lidar_sensor/material_color

Further information can be obtained about topics, for instance, the type of message that flows through them. We can check
what is the message type used in one of the sensors as follows.

.. code-block::

    gz topic -i --topic /rgbd_camera/image

The output of that command should be similar to the following.

.. code-block:: console

    Publishers [Address, Message Type]:
      tcp://172.16.191.128:40679, gz.msgs.Image
    Subscribers [Address, Message Type]:
      tcp://172.16.191.128:40229, gz.msgs.Image

Information about services, similarly, can be obtained with the following command.

.. code-block::

    gz service -l

Resulting in the following output.

.. dropdown:: Services in ``sensors_demo.sdf``

    .. code-block:: console

        /camera/set_rate
        /camera_alone/set_rate
        /depth_camera/set_rate
        /gazebo/resource_paths/add
        /gazebo/resource_paths/get
        /gazebo/resource_paths/resolve
        /gazebo/worlds
        /gui/camera/view_control
        /gui/camera/view_control/reference_visual
        /gui/camera/view_control/sensitivity
        /gui/follow
        /gui/follow/offset
        /gui/move_to
        /gui/move_to/pose
        /lidar/set_rate
        /rgbd_camera/set_rate
        /sensors/marker
        /sensors/marker/list
        /sensors/marker_array
        /server_control
        /thermal_camera/set_rate
        /world/lidar_sensor/control
        /world/lidar_sensor/control/state
        /world/lidar_sensor/create
        /world/lidar_sensor/create/blocking
        /world/lidar_sensor/create_multiple
        /world/lidar_sensor/create_multiple/blocking
        /world/lidar_sensor/declare_parameter
        /world/lidar_sensor/disable_collision
        /world/lidar_sensor/disable_collision/blocking
        /world/lidar_sensor/enable_collision
        /world/lidar_sensor/enable_collision/blocking
        /world/lidar_sensor/entity/system/add
        /world/lidar_sensor/generate_world_sdf
        /world/lidar_sensor/get_parameter
        /world/lidar_sensor/gui/info
        /world/lidar_sensor/level/set_performer
        /world/lidar_sensor/light_config
        /world/lidar_sensor/light_config/blocking
        /world/lidar_sensor/list_parameters
        /world/lidar_sensor/playback/control
        /world/lidar_sensor/remove
        /world/lidar_sensor/remove/blocking
        /world/lidar_sensor/scene/graph
        /world/lidar_sensor/scene/info
        /world/lidar_sensor/set_parameter
        /world/lidar_sensor/set_physics
        /world/lidar_sensor/set_physics/blocking
        /world/lidar_sensor/set_pose
        /world/lidar_sensor/set_pose/blocking
        /world/lidar_sensor/set_pose_vector
        /world/lidar_sensor/set_pose_vector/blocking
        /world/lidar_sensor/set_spherical_coordinates
        /world/lidar_sensor/set_spherical_coordinates/blocking
        /world/lidar_sensor/state
        /world/lidar_sensor/state_async
        /world/lidar_sensor/system/info
        /world/lidar_sensor/visual_config
        /world/lidar_sensor/visual_config/blocking
        /world/lidar_sensor/wheel_slip
        /world/lidar_sensor/wheel_slip/blocking

We can also obtain information about each service. For instance, with the command.

.. code-block::

    gz service -i --service /rgbd_camera/set_rate

Resulting in the following output.

.. code-block::

    Service providers [Address, Request Message Type, Response Message Type]:
      tcp://172.16.191.128:36013, gz.msgs.Double, gz.msgs.Empty

There are multiple ways to interact with the :program:`Gazebo` internal topics. This can be, for instance, done
through the commandline using similar commands to the ones we have shown so far. Nonetheless, if you have spent
so much time learning :program:`ROS2`, it would be more convenient to find a way to **bridge** the topics and services
from :program:`Gazebo` and :program:`ROS2`.

Nonetheless, when some piece of information is not flowing as expected, remember these commands to help troubleshoot.

The package :file:`ros_gz_sim`
------------------------------

.. seealso::

    The official repository: https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim

The package :file:`ros_gz_sim` has a few tools allowing us to control :program:`Gazebo` over :program:`ROS2`.
Namely, the ability to start :program:`Gazebo` and spawn (add) objects. These might depend on :program:`Gazebo`
topics and services, therefore make sure that the necessary plugins are enabled.

Launching :program:`Gazebo`
+++++++++++++++++++++++++++

For example, we can launch :program:`Gazebo` directly from a launch file.

.. code-block:: console

    ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=shapes.sdf

In this command, ``gz_args`` we add a representation file known to :program:`Gazebo`. In this case, ``shapes.sdf``.
You can specify other scenes.

Spawn models
++++++++++++

You can use the following command to spawn models in :program:`Gazebo`.

.. tab-set::

    .. tab-item:: Terminal 1: Run :program:`Gazebo`

        .. code-block:: console

            ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=shapes.sdf

    .. tab-item:: Terminal 2: Spawn model

        .. code-block:: console

            ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=shapes file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5

        As arguments we have the

        - model filename
        - name for the entity inside :program:`Gazebo`
        - position coordinates of the model

.. note::

    The scene specified in the argument ``world`` must be active in :program:`Gazebo` for the command above to work.

This will add the model, in this case, :file:`model.sdf` to the scene :file:`shapes.sdf`. The output on the terminal
will be as follows.

.. code-block:: console

    [INFO] [launch]: All log files can be found below /home/murilo/.ros/log/2025-11-06-11-19-14-321361-murilo-VMware20-1-14158
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [create-1]: process started with pid [14161]
    [create-1] [WARN] [1762427959.367373569] [ros_gz_sim]: Waiting for service [/world/shapes/create] to become available ...
    [create-1] [INFO] [1762427959.598035759] [ros_gz_sim]: Entity creation successful.
    [INFO] [create-1]: process has finished cleanly [pid 14161]

The package :file:`ros_gz_sim_demos`
-----------------------------------

.. seealso::

    The official repository: https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos

This official package has many examples integrating :program:`Gazebo` and :program:`ROS2`. It showcases many functionalities
that are useful for robotics applications.

You can verify all files usable for :program:`Gazebo` demos in the following folder.

.. code-block:: console

    cd $(ros2 pkg prefix --share ros_gz_sim_demos)
    nautilus .

The current file structure is shown in the dropdown below. There are many launch files, :program:`rviz` configuration files, and a few
:file:`.sdf` models.

.. dropdown:: The structure of ``ros_gz_sim_demos``.

    .. code-block:: console

        /opt/ros/jazzy/share/ros_gz_sim_demos
        ├── cmake
        │   ├── ros_gz_sim_demosConfig.cmake
        │   └── ros_gz_sim_demosConfig-version.cmake
        ├── environment
        │   ├── ament_prefix_path.dsv
        │   ├── ament_prefix_path.sh
        │   ├── path.dsv
        │   ├── path.sh
        │   └── ros_gz_sim_demos.dsv
        ├── launch
        │   ├── air_pressure.launch.py
        │   ├── battery.launch.py
        │   ├── camera.launch.py
        │   ├── depth_camera.launch.py
        │   ├── diff_drive.launch.py
        │   ├── gpu_lidar_bridge.launch.py
        │   ├── gpu_lidar.launch.py
        │   ├── image_bridge.launch.py
        │   ├── imu.launch.py
        │   ├── joint_states.launch.py
        │   ├── magnetometer.launch.py
        │   ├── navsat_gpsfix.launch.py
        │   ├── navsat.launch.py
        │   ├── rgbd_camera_bridge.launch.py
        │   ├── rgbd_camera.launch.py
        │   ├── robot_description_publisher.launch.py
        │   ├── sdf_parser.launch.py
        │   ├── tf_bridge.launch.py
        │   └── triggered_camera.launch.py
        ├── local_setup.bash
        ├── local_setup.dsv
        ├── local_setup.sh
        ├── local_setup.zsh
        ├── models
        │   ├── cardboard_box
        │   │   ├── materials
        │   │   │   └── textures
        │   │   │       └── cardboard_box.png
        │   │   ├── meshes
        │   │   │   └── cardboard_box.dae
        │   │   ├── model.config
        │   │   ├── model.sdf
        │   │   └── thumbnails
        │   │       ├── 1.png
        │   │       ├── 2.png
        │   │       ├── 3.png
        │   │       ├── 4.png
        │   │       └── 5.png
        │   ├── double_pendulum_model.sdf
        │   ├── rrbot.xacro
        │   └── vehicle
        │       ├── model.config
        │       └── model.sdf
        ├── package.dsv
        ├── package.xml
        ├── rviz
        │   ├── camera.rviz
        │   ├── depth_camera.rviz
        │   ├── diff_drive.rviz
        │   ├── gpu_lidar_bridge.rviz
        │   ├── gpu_lidar.rviz
        │   ├── imu.rviz
        │   ├── joint_states.rviz
        │   ├── rgbd_camera_bridge.rviz
        │   ├── rgbd_camera.rviz
        │   ├── robot_description_publisher.rviz
        │   ├── tf_bridge.rviz
        │   └── vehicle.rviz
        └── worlds
            ├── default.sdf
            └── vehicle.sdf

.. note::

    The contents above are not retrieved automatically so they might not represent the latest version of the repository.
    See https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos .

The :file:`camera.launch.py` demo
+++++++++++++++++++++++++++++++++

We can run this example with the following command.

.. code-block:: console

    ros2 launch ros_gz_sim_demos camera.launch.py

This will show :program:`Gazebo`, with a couple of objects and a camera. The camera view is being rendered
by :program:`Gazebo`. The simulation is started. At the same time, :program:`rviz2` is executed. This demo is
important because cameras are difficult to simulate otherwise and being able to access them via :program:`ROS2`
allows you to create powerful image-based robot controllers and planners.

We can take a look at the topics created, while those programs are running, with the following command.

.. code-block:: console

    ros2 topic list

This will output the following.

.. code-block:: console

    /camera
    /camera_info
    /clicked_point
    /initialpose
    /move_base_simple/goal
    /parameter_events
    /rosout
    /tf
    /tf_static

The launch file, :file:`camera.launch.py`, is currently performing the following steps.

- Starting :program:`Gazebo` with the built-in scene :file:`camera_sensor.sdf`.
- Running :program:`rviz2` with the configuration file :file:`camera.rviz`.
- Running ``ros_gz_bridge`` with the correct parameters to expose the camera sensor from :program:`Gazebo` to :program:`ROS2`.

This demo shows the camera simulation results on :program:`rviz2` because it's possibly the most convenient way of
showing that the information is correctly flowing through :program:`ROS2`.

.. note::

    Some unused topics such as ``/tf`` are defined in :file:`camera.rviz`, but don't worry about those.

Below are the contents of `the launch file <https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/launch/camera.launch.py>`_.

.. dropdown:: The contents of :file:`launch/camera.launch.py`

    .. rli:: https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/launch/camera.launch.py
        :language: python
        :linenos:

Below are the contents of `the rviz file <https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/rviz/camera.rviz>`_.

.. dropdown:: The contents of :file:`rviz/camera.rviz`

    .. rli:: https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/rviz/camera.rviz
        :language: yaml
        :linenos:

The launch file uses the package :file:`ros_gz_bridge`, which is used to create the interfaces,
for instance topics, between :program:`Gazebo` and :program:`ROS2`.

We will see this package in more detail in the following section.


References
----------

    The official documentation for :program:`Gazebo` is overall good. Here are some main topics where this tutorial
    borrowed from.


    - https://gazebosim.org/docs/harmonic/sensors/
    - https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/
    - https://gazebosim.org/docs/harmonic/ros2_integration/
    - https://gazebosim.org/docs/harmonic/ros2_spawn_model/
    - https://gazebosim.org/docs/harmonic/ros2_interop/
    - https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/
    - https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md

