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
and direct changes in :file:`.sdf` files. The format is :file:`xml`, which should be a familiar file format by now.


This example is `available in the official repository <https://raw.githubusercontent.com/gazebosim/gz-sim/refs/heads/gz-sim8/examples/worlds/shapes.sdf>`_.

.. dropdown:: shapes.sdf

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

This official package has many examples integrating :program:`Gazebo` and :program:`ROS2`. It showcases many functionalities
that are useful for robotics applications.

You can verify all files usable for :program:`Gazebo` demos in the following folder.

.. code-block:: console

    cd $(ros2 pkg prefix --share ros_gz_sim_demos)
    nautilus .

The current file structure is shown in the dropdown below. There are many launch files, :program:`rviz` configuration files, and a few
:file:`.sdf` models.

.. note::

    The contents below are not retrieved automatically so they might not represent the latest version of the repository.
    See https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos .

.. dropdown:: ros_gz_sim_demos

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

Below are the contents of `the launch file <https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/launch/camera.launch.py>`_.

.. dropdown:: ros_gz_sim_demos/launch/camera.launch.py

    .. rli:: https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/launch/camera.launch.py
        :language: python
        :linenos:

Below are the contents of `the rviz file <https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/rviz/camera.rviz>`_.

.. dropdown:: ros_gz_sim_demos/rviz/camera.rviz

    .. rli:: https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_sim_demos/rviz/camera.rviz
        :language: yaml
        :linenos:

The launch file mentions the package :file:`ros_gz_bridge`, described below, which is used to create the interfaces,
for instance topics, between :program:`Gazebo` and :program:`ROS2`.

Gazebo topics and services
--------------------------

.. caution::

    Internal :program:`Gazebo` topics and services are **not** the same as :program:`ROS2` topics and services.

A :program:`Gazebo` scene will have a number of internal topics and services. For instance, let us refer to the following
scene.

.. dropdown:: sensors_demo.sdf

    .. rli:: https://raw.githubusercontent.com/gazebosim/gz-sim/refs/heads/gz-sim8/examples/worlds/sensors_demo.sdf
        :language: xml

For this section, suppose that we have the scene always open with

.. code-block:: console

    gz sim sensors_demo.sdf

A frequently used tool to allow us to inspect internal Gazebo topics will be, in another terminal,

.. code-block:: console

    gz topic -l

With the following result for this scene

.. dropdown:: Result of :program:`gz topic -l` for ``sensors_demo.sdf``

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

This results in

.. code-block:: console

    Publishers [Address, Message Type]:
      tcp://172.16.191.128:40679, gz.msgs.Image
    Subscribers [Address, Message Type]:
      tcp://172.16.191.128:40229, gz.msgs.Image

Information about services, similarly, can be obtained with the following command.

.. code-block::

    gz service -l

Resulting in the following output.

.. dropdown:: Result of :program:`gz service -l` for ``sensors_demo.sdf``

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
so much time larning :program:`ROS2`, it would be more convenient to find a way to **bridge** the topics and services
from :program:`Gazebo` and :program:`ROS2`. This motivates the following section.

Using ``ros_gz_bridge``
-----------------------

.. caution::

    Internal :program:`Gazebo` topics are not the same as :program:`ROS2` topics. We must use ``ros_gz_bridge`` to
    expose the interfaces through :program:`ROS2`.

Each :program:`Gazebo` message must be paired with a correct :program:`ROS2` message if you want to access these outside :program:`Gazebo`.
The file `ros_gz_bridge/README.md <https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_bridge/README.md>`_
in the official documentation shows the mappings.

Here is the current table.

.. rli:: https://raw.githubusercontent.com/gazebosim/ros_gz/refs/heads/jazzy/ros_gz_bridge/README.md
    :language: markdown
    :lines: 6-82

We will be using mostly :program:`parameter_bridge`, which is part of :file:`ros_gz_bridge` to make these connections.
More information about the tool can be obtained with the help command.

.. code-block::

    ros2 run ros_gz_bridge parameter_bridge -h

To summarise the output, it is expected that we run

    ros2 run ros_gz_bridge \
    parameter_bridge \
    ``<Gazebo Topic>``\ @\ ``<ROS Type>``\ @\ ``<Gazebo Transport Type>``

Using the logic above, we have that

- ``<Gazebo Topic>`` defined for the entity on Gazebo and can be obtained with ``gz topic -l``.
- We can find ``<Gazebo Transport Type>`` with ``gz topic -i --topic <Gazebo Topic>``.
- Lastly, we check the pairing table and see what ``<ROS Type>`` matches the ``<Gazebo Transport Type>``.

Let's see some examples.

Sensors
+++++++

Simulated sensor information is becoming evermore useful. In simulators, one of the main aspects making them useful
will be our ability to obtain their data through :program:`ROS2`. Cameras are likely to be the most common, but
other sensors, such as lidars, are frequently used.

For each of these subsections, suppose that we have the :file:`sensors_demo.sdf` scene always open. Don't forget to
start the simulation as well!

.. code-block:: console

    gz sim sensors_demo.sdf

.. note::

    Sensor information will only start to be published after the simulation is started in :program:`Gazebo`.

gz.msgs.Image
~~~~~~~~~~~~~

In a previous section, we have seen that the :program:`Gazebo` topic ``/rgbd_camera/image`` exists and has
transport type ``gz.msgs.Image``. Looking at the pairing table, we notice that the pairing :program:`ROS2` message
type is ``sensor_msgs/msg/Image``.

We can run, therefore in one terminal, the following command.

.. code-block:: console

    ros2 run ros_gz_bridge \
    parameter_bridge \
    /rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image

To show the images, we can use :program:`rqt_image_view`. Notice that the internal :program:`Gazebo` topic name
will be replicated into a :program:`ROS2` topic with the same name.

.. code-block:: console

    ros2 run rqt_image_view rqt_image_view /rgbd_camera/image

This will show the image obtained from :program:`Gazebo` in :program:`rqt_image_view`, effectively showing that these
two are paired.

.. admonition:: Exercises

    Can you do the same for the ``/depth_camera`` or ``/thermal_camera`` internal :program:`Gazebo` topic available in the same scene?
    What steps would you take to show the images available in :program:`Gazebo` into :program:`rqt_image_view`?

gz.msgs.LaserScan
~~~~~~~~~~~~~~~~~

We have already listed the :program:`Gazebo` topics for this scene, so we know that there is a topic called
``\lidar``. We can obtain the related information with the following command.

.. code-block::

    gz topic -i --topic /lidar

This will output the following.

.. code-block::

    Publishers [Address, Message Type]:
      tcp://172.16.191.128:40679, gz.msgs.LaserScan
    No subscribers on topic [/lidar]

By looking at the pairing table, we find that ``gz.msgs.LaserScan`` should be paired with a ``sensor_msgs/msg/LaserScan``.
Therefore, we can run the bridge as follows.

.. code-block:: console

    ros2 run ros_gz_bridge \
    parameter_bridge \
    /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan

One convenient way to visualise this messages is through :program:`rviz2`. We can do so as follows.

.. code-block:: console

    ros2 run rviz2 rviz2 -f camera_with_lidar/link/gpu_lidar

.. note::

    We are starting :program:`rviz2` with the reference frame ``camera_with_lidar/link/gpu_lidar`` because that is
    the frame shown in the topic ``/lidar``.

We can then define the proper :program:`rviz2` view so be able to visualise the laser scan results.

#. :menuselection:`Add --> rviz_default_plugins --> LaserScan --> OK`.
#. :menuselection:`Displays --> LaserScan --> topic --> \lidar --> Press ENTER`.

Getting entity pose information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

One important aspect of entities in any simulation is their pose. We must be able to obtain them to define the behavior
of our robots. For instance, if a box in a simulation moves, the controller might need this information to define
evasive maneuvers.

The most basic manner to be able to obtain the pose of an entity externally is to add the following to a model, in
the :file:`.sdf` scene.

.. code-block:: xml

    <plugin
            filename="gz-sim-pose-publisher-system"
            name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose>
    </plugin>

.. note::

    Settings will vary for more advanced use, but for basic shapes this will be enough.

To show how this works, let us create a folder for our modified :file:`shapes.sdf` scene.

.. code-block:: console

    mkdir -p ~/gazebo_tutorial_workspace/scenes
    cd ~/gazebo_tutorial_workspace/scenes

In this case, the file is quite large, so I suggest downloading :file:`shapes_with_pose_publisher.sdf` and adding it to the folder above.

:download:`shapes_with_pose_publisher.sdf <../../../gazebo_tutorial_workspace/scenes/shapes_with_pose_publisher.sdf>`

.. dropdown:: Contents of :file:`shapes_with_pose_publisher.sdf`

    You will note that this is simply the default :file:`shapes.sdf` with the plugin added to each entity.

    .. literalinclude:: ../../../gazebo_tutorial_workspace/scenes/shapes_with_pose_publisher.sdf
       :language: xml
       :linenos:
       :emphasize-lines: 95-102,144-151,191-198,239-246,285-292,326-333

We open :program:`Gazebo` with this scene, as follows, then run the simulation by clicking the run button.

.. code-block:: console

    gz sim ~/gazebo_tutorial_workspace/scenes/shapes_with_pose_publisher.sdf

.. note::

    Don't forget to run the simulation otherwise most information will not be available.

The plugin for each entity will create a ``pose`` topic. We can verify that with the following command.

.. code-block:: console

    gz topic -l

The result will be as follows, where the relevant topics are highlighted.

.. code-block:: console
    :emphasize-lines: 6-11

    /clock
    /gazebo/resource_paths
    /gui/camera/pose
    /gui/currently_tracked
    /gui/track
    /model/box/pose
    /model/capsule/pose
    /model/cone/pose
    /model/cylinder/pose
    /model/ellipsoid/pose
    /model/sphere/pose
    /stats
    /world/shapes_with_pose_publisher/clock
    /world/shapes_with_pose_publisher/dynamic_pose/info
    /world/shapes_with_pose_publisher/pose/info
    /world/shapes_with_pose_publisher/scene/deletion
    /world/shapes_with_pose_publisher/scene/info
    /world/shapes_with_pose_publisher/state
    /world/shapes_with_pose_publisher/stats
    /world/shapes_with_pose_publisher/light_config
    /world/shapes_with_pose_publisher/material_color

As always, we can investigate the :program:`Gazebo` type these topics, as we expect all of them to be the same as they
come from the same plugin. Let's choose the *box* as it's the first entity on the list.

We use the following command.

.. code-block:: console

    gz topic -i --topic /model/box/pose

It will output the type.

.. code-block:: console

    Publishers [Address, Message Type]:
      tcp://172.16.191.128:41611, gz.msgs.Pose
    No subscribers on topic [/model/box/pose]

Because this will be an unilateral bridge from :program:`Gazebo` to :program:`ROS2`, we use the ``[`` instead of ``@``
as follows.

.. code-block:: console

    ros2 run ros_gz_bridge \
    parameter_bridge \
    /model/box/pose@geometry_msgs/msg/Pose[gz.msgs.Pose

We can see the contents of the topic with the following command. It will show the real-time information of the entity,
even if you move it around within Gazebo.

.. code-block:: console

    ros2 topic echo /model/box/pose

We can see how fast your bridge is running with the usual command below.

.. code-block:: console

    ros2 topic hz /model/box/pose

Depending on the quality of your machine, your results will very, but you should expect this frequency to be quite
high.

.. code-block:: console

    average rate: 799.066
        min: 0.000s max: 0.009s std dev: 0.00142s window: 800
    average rate: 824.706
        min: 0.000s max: 0.009s std dev: 0.00129s window: 1652
    average rate: 811.552
        min: 0.000s max: 0.010s std dev: 0.00131s window: 2439
    average rate: 815.633
        min: 0.000s max: 0.010s std dev: 0.00129s window: 3267

.. admonition:: Exercises

    I have shown how to obtain the pose of the ``box`` entity. This scene has other five topics, namely.

    - /model/capsule/pose
    - /model/cone/pose
    - /model/cylinder/pose
    - /model/ellipsoid/pose
    - /model/sphere/pose

    Can you run the bridge of each one of these and see the results of each of these entities?

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

