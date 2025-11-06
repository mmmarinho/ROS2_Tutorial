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

Check the guides below for basic functionality. I advise starting by choosing the file :file:`ackermann_steering.sdf` in these plugins.

- `Apply force torque plugin <https://gazebosim.org/api/sim/8/apply_force_torque.html>`_.
- `Mouse drag plugin <https://gazebosim.org/api/sim/8/mouse_drag.html>`_.

World files :file:`.sdf`
------------------------

Even simple world files can have a relatively complex format. Even :file:`shapes.sdf`, that has only a few simple
objects, is difficult to parse visually. For most things, it is easier to use :program:`Gazebo` itself and limit
and direct changes in :file:`.sdf` files.

.. dropdown:: Contents of :file:`shapes.sdf`

    This example is `available in the official repository <https://raw.githubusercontent.com/gazebosim/gz-sim/refs/heads/gz-sim8/examples/worlds/shapes.sdf>`_.

    .. rli:: https://raw.githubusercontent.com/gazebosim/gz-sim/refs/heads/gz-sim8/examples/worlds/shapes.sdf
        :language: xml

.. important::

    If your application demands it, you will need specialised programs to create 3D models, such as `Blender <https://www.blender.org/>`_.
    It won't be usually manageable to create complicated models directly on :program:`Gazebo`.

The package :file:`ros_gz_sim`
------------------------------

The package :file:`ros_gz_sim` has a set of tools allowing us to control :program:`Gazebo` over :program:`ROS2`.

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

            os2 launch ros_gz_sim gz_sim.launch.py gz_args:=shapes.sdf

    .. tab-item:: Terminal 2: Spawn model

        Note that as arguments we have the model file, a name, and the position of the model.

        .. code-block:: console

            ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=shapes file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5

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

This official package has many examples integrating :program:`Gazebo` and :program:`ROS2`. It showcases many functionalities
that are useful for robotics applications.

You can verify all files usable for :program:`Gazebo` demos in the following folder.

.. code-block:: console

    cd $(ros2 pkg prefix --share ros_gz_sim_demos)
    nautilus .

The current file structure is shown in the dropdown below. There are many launch files, :program:`rviz` configuration files, and a few
:file:`.sdf` models.

.. dropdown:: Contents of :file:`ros_gz_sim_demos`

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

These are the contents of `the launch file <https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/launch/camera.launch.py>`_.

.. rli:: https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/launch/camera.launch.py
    :language: python
    :linenos:

The launch file mentions the package :file:`ros_gz_bridge`, described below.

Using ``ros_gz_bridge``
-----------------------





References
----------

    The official documentation for ``gazebo`` is very good. Here are some main topics where this tutorial
    borrowed from.


    - https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/
    - https://gazebosim.org/docs/harmonic/ros2_integration/
    - https://gazebosim.org/docs/harmonic/ros2_spawn_model/
    - https://gazebosim.org/docs/harmonic/ros2_interop/
    - https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/

