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

    .. code-block:: xml

        <sdf version='1.10'>
          <world name='shapes'>
            <scene>
              <ambient>1 1 1 1</ambient>
              <background>0.800000012 0.800000012 0.800000012 1</background>
              <shadows>true</shadows>
            </scene>
            <gravity>0 0 -9.8000000000000007</gravity>
            <magnetic_field>5.5644999999999998e-06 2.2875799999999999e-05 -4.2388400000000002e-05</magnetic_field>
            <atmosphere type='adiabatic'/>
            <physics type='ode'>
              <max_step_size>0.001</max_step_size>
              <real_time_factor>1</real_time_factor>
              <real_time_update_rate>1000</real_time_update_rate>
            </physics>
            <model name='ground_plane'>
              <static>true</static>
              <link name='link'>
                <collision name='collision'>
                  <geometry>
                    <plane>
                      <normal>0 0 1</normal>
                      <size>100 100</size>
                    </plane>
                  </geometry>
                  <surface>
                    <friction>
                      <ode/>
                    </friction>
                    <bounce/>
                    <contact/>
                  </surface>
                </collision>
                <visual name='visual'>
                  <geometry>
                    <plane>
                      <normal>0 0 1</normal>
                      <size>100 100</size>
                    </plane>
                  </geometry>
                  <material>
                    <ambient>0.800000012 0.800000012 0.800000012 1</ambient>
                    <diffuse>0.800000012 0.800000012 0.800000012 1</diffuse>
                    <specular>0.800000012 0.800000012 0.800000012 1</specular>
                  </material>
                </visual>
                <pose>0 0 0 0 0 0</pose>
                <inertial>
                  <pose>0 0 0 0 0 0</pose>
                  <mass>1</mass>
                  <inertia>
                    <ixx>1</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>1</iyy>
                    <iyz>0</iyz>
                    <izz>1</izz>
                  </inertia>
                </inertial>
                <enable_wind>false</enable_wind>
              </link>
              <pose>0 0 0 0 0 0</pose>
              <self_collide>false</self_collide>
            </model>
            <model name='box'>
              <pose>0 0 0.5 0 0 0</pose>
              <link name='box_link'>
                <inertial>
                  <inertia>
                    <ixx>0.16666</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.16666</iyy>
                    <iyz>0</iyz>
                    <izz>0.16666</izz>
                  </inertia>
                  <mass>1</mass>
                  <pose>0 0 0 0 0 0</pose>
                </inertial>
                <collision name='box_collision'>
                  <geometry>
                    <box>
                      <size>1 1 1</size>
                    </box>
                  </geometry>
                  <surface>
                    <friction>
                      <ode/>
                    </friction>
                    <bounce/>
                    <contact/>
                  </surface>
                </collision>
                <visual name='box_visual'>
                  <geometry>
                    <box>
                      <size>1 1 1</size>
                    </box>
                  </geometry>
                  <material>
                    <ambient>1 0 0 1</ambient>
                    <diffuse>1 0 0 1</diffuse>
                    <specular>1 0 0 1</specular>
                  </material>
                </visual>
                <pose>0 0 0 0 0 0</pose>
                <enable_wind>false</enable_wind>
              </link>
              <static>false</static>
              <self_collide>false</self_collide>
            </model>
            <model name='cylinder'>
              <pose>0 -1.5 0.5 0 0 0</pose>
              <link name='cylinder_link'>
                <inertial>
                  <inertia>
                    <ixx>0.14580000000000001</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.14580000000000001</iyy>
                    <iyz>0</iyz>
                    <izz>0.125</izz>
                  </inertia>
                  <mass>1</mass>
                  <pose>0 0 0 0 0 0</pose>
                </inertial>
                <collision name='cylinder_collision'>
                  <geometry>
                    <cylinder>
                      <radius>0.5</radius>
                      <length>1</length>
                    </cylinder>
                  </geometry>
                  <surface>
                    <friction>
                      <ode/>
                    </friction>
                    <bounce/>
                    <contact/>
                  </surface>
                </collision>
                <visual name='cylinder_visual'>
                  <geometry>
                    <cylinder>
                      <radius>0.5</radius>
                      <length>1</length>
                    </cylinder>
                  </geometry>
                  <material>
                    <ambient>0 1 0 1</ambient>
                    <diffuse>0 1 0 1</diffuse>
                    <specular>0 1 0 1</specular>
                  </material>
                </visual>
                <pose>0 0 0 0 0 0</pose>
                <enable_wind>false</enable_wind>
              </link>
              <static>false</static>
              <self_collide>false</self_collide>
            </model>
            <model name='sphere'>
              <pose>0 1.5 0.5 0 0 0</pose>
              <link name='sphere_link'>
                <inertial>
                  <inertia>
                    <ixx>0.10000000000000001</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.10000000000000001</iyy>
                    <iyz>0</iyz>
                    <izz>0.10000000000000001</izz>
                  </inertia>
                  <mass>1</mass>
                  <pose>0 0 0 0 0 0</pose>
                </inertial>
                <collision name='sphere_collision'>
                  <geometry>
                    <sphere>
                      <radius>0.5</radius>
                    </sphere>
                  </geometry>
                  <surface>
                    <friction>
                      <ode/>
                    </friction>
                    <bounce/>
                    <contact/>
                  </surface>
                </collision>
                <visual name='sphere_visual'>
                  <geometry>
                    <sphere>
                      <radius>0.5</radius>
                    </sphere>
                  </geometry>
                  <material>
                    <ambient>0 0 1 1</ambient>
                    <diffuse>0 0 1 1</diffuse>
                    <specular>0 0 1 1</specular>
                  </material>
                </visual>
                <pose>0 0 0 0 0 0</pose>
                <enable_wind>false</enable_wind>
              </link>
              <static>false</static>
              <self_collide>false</self_collide>
            </model>
            <model name='capsule'>
              <pose>0 -3 0.5 0 0 0</pose>
              <link name='capsule_link'>
                <inertial>
                  <inertia>
                    <ixx>0.074153999999999998</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.074153999999999998</iyy>
                    <iyz>0</iyz>
                    <izz>0.018769000000000001</izz>
                  </inertia>
                  <mass>1</mass>
                  <pose>0 0 0 0 0 0</pose>
                </inertial>
                <collision name='capsule_collision'>
                  <geometry>
                    <capsule>
                      <radius>0.20000000000000001</radius>
                      <length>0.59999999999999998</length>
                    </capsule>
                  </geometry>
                  <surface>
                    <friction>
                      <ode/>
                    </friction>
                    <bounce/>
                    <contact/>
                  </surface>
                </collision>
                <visual name='capsule_visual'>
                  <geometry>
                    <capsule>
                      <radius>0.20000000000000001</radius>
                      <length>0.59999999999999998</length>
                    </capsule>
                  </geometry>
                  <material>
                    <ambient>1 1 0 1</ambient>
                    <diffuse>1 1 0 1</diffuse>
                    <specular>1 1 0 1</specular>
                  </material>
                </visual>
                <pose>0 0 0 0 0 0</pose>
                <enable_wind>false</enable_wind>
              </link>
              <static>false</static>
              <self_collide>false</self_collide>
            </model>
            <model name='ellipsoid'>
              <pose>0 3 0.5 0 0 0</pose>
              <link name='ellipsoid_link'>
                <inertial>
                  <inertia>
                    <ixx>0.068000000000000005</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.058000000000000003</iyy>
                    <iyz>0</iyz>
                    <izz>0.025999999999999999</izz>
                  </inertia>
                  <mass>1</mass>
                  <pose>0 0 0 0 0 0</pose>
                </inertial>
                <collision name='ellipsoid_collision'>
                  <geometry>
                    <ellipsoid>
                      <radii>0.20000000000000001 0.29999999999999999 0.5</radii>
                    </ellipsoid>
                  </geometry>
                  <surface>
                    <friction>
                      <ode/>
                    </friction>
                    <bounce/>
                    <contact/>
                  </surface>
                </collision>
                <visual name='ellipsoid_visual'>
                  <geometry>
                    <ellipsoid>
                      <radii>0.20000000000000001 0.29999999999999999 0.5</radii>
                    </ellipsoid>
                  </geometry>
                  <material>
                    <ambient>1 0 1 1</ambient>
                    <diffuse>1 0 1 1</diffuse>
                    <specular>1 0 1 1</specular>
                  </material>
                </visual>
                <pose>0 0 0 0 0 0</pose>
                <enable_wind>false</enable_wind>
              </link>
              <static>false</static>
              <self_collide>false</self_collide>
            </model>
            <model name='cone'>
              <pose>0 4.5 0.5 0 0 0</pose>
              <link name='cone_link'>
                <inertial auto='true'>
                  <density>1</density>
                  <pose>0 0 -0.25 0 0 0</pose>
                  <mass>0.26179938779914941</mass>
                  <inertia>
                    <ixx>0.019634954084936207</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.019634954084936207</iyy>
                    <iyz>0</iyz>
                    <izz>0.019634954084936207</izz>
                  </inertia>
                </inertial>
                <collision name='cone_collision'>
                  <geometry>
                    <cone>
                      <radius>0.5</radius>
                      <length>1</length>
                    </cone>
                  </geometry>
                  <surface>
                    <friction>
                      <ode/>
                    </friction>
                    <bounce/>
                    <contact/>
                  </surface>
                </collision>
                <visual name='cone_visual'>
                  <geometry>
                    <cone>
                      <radius>0.5</radius>
                      <length>1</length>
                    </cone>
                  </geometry>
                  <material>
                    <ambient>1 0.469999999 0 1</ambient>
                    <diffuse>1 0.469999999 0 1</diffuse>
                    <specular>1 0.469999999 0 1</specular>
                  </material>
                </visual>
                <pose>0 0 0 0 0 0</pose>
                <enable_wind>false</enable_wind>
              </link>
              <static>false</static>
              <self_collide>false</self_collide>
            </model>
            <light name='sun' type='directional'>
              <pose>0 0 10 0 0 0</pose>
              <cast_shadows>true</cast_shadows>
              <intensity>1</intensity>
              <direction>-0.5 0.10000000000000001 -0.90000000000000002</direction>
              <diffuse>0.800000012 0.800000012 0.800000012 1</diffuse>
              <specular>0.200000003 0.200000003 0.200000003 1</specular>
              <attenuation>
                <range>1000</range>
                <linear>0.01</linear>
                <constant>0.90000000000000002</constant>
                <quadratic>0.001</quadratic>
              </attenuation>
              <spot>
                <inner_angle>0</inner_angle>
                <outer_angle>0</outer_angle>
                <falloff>0</falloff>
              </spot>
            </light>
          </world>
        </sdf>


Gazebo from ROS2
----------------

.. code-block:: console

    ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=shapes.sdf

In this command, ``gz_args`` we add a representation file that exists within the :program:`Gazebo` paths. In this case,
``empty.sdf``.

The :file:`ros_gz_sim_demos` folder
------------------------------------

You can verify all files usable for :program:`Gazebo` demos in the following folder.

.. code-block:: console

    cd $(ros2 pkg prefix --share ros_gz_sim_demos)
    nautilus .

Currently, this is the file structure. There are many launch files, :program:`rviz` configuration files, and a few
:file:`.sdf` models.

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


Using ``ros_gz_bridge``
-----------------------



Spawn models
++++++++++++

You can use the following command to spawn models in :program:`Gazebo`.

.. code-block:: console

    ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=shapes file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5

This will add the model, in this case, :file:`model.sdf` to the scene :file:`shapes.sdf`. The output on the terminal
will be as follows.

.. code-block:: console

    [INFO] [launch]: All log files can be found below /home/murilo/.ros/log/2025-11-06-11-19-14-321361-murilo-VMware20-1-14158
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [create-1]: process started with pid [14161]
    [create-1] [WARN] [1762427959.367373569] [ros_gz_sim]: Waiting for service [/world/shapes/create] to become available ...
    [create-1] [INFO] [1762427959.598035759] [ros_gz_sim]: Entity creation successful.
    [INFO] [create-1]: process has finished cleanly [pid 14161]


References
----------

    The official documentation for ``gazebo`` is very good. Here are some main topics where this tutorial
    borrowed from.


    - https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/
    - https://gazebosim.org/docs/harmonic/ros2_integration/
    - https://gazebosim.org/docs/harmonic/ros2_spawn_model/
    - https://gazebosim.org/docs/harmonic/ros2_interop/
    - https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/

