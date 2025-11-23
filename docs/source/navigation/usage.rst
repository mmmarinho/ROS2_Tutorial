Using ``nav2``
==============

.. versionadded:: Jazzy

   This section.

.. include:: ../the_topic_is_under_heavy_construction.rst

There is a plethora of online examples of ``nav2``, with varying levels of detail and quality. My approach will top-down,
where we will see how illustrative examples and then walk through their contents.

Navigation on a known map
+++++++++++++++++++++++++

.. seealso::

    Official documentation: https://docs.nav2.org/tutorials/docs/navigation2_on_real_turtlebot3.html

In this example, our interest is looking at how navigation can be done with built-in tools, using a known *map*. A
map will have many definitions and the literature about it is extensive. For now, we can think of the map as something
usual. It points out where things are, for instance, where passable regions (such as roads) are, and possibly hazards,
obstacles, and other relevant objects.

In this example, a `TurtleBot3 <https://www.turtlebot.com/turtlebot3/>`_ will be used. As part of ``nav2_bringup``, there
is a rather complete example that we can utilize, namely :file:`tb3_simulation_launch.py`.

.. warning::

    The ``sigterm_timeout`` flag is particularly important. Given that some devices we use might not be powerful enough
    for all processes to finish cleanly within 5 or 10 seconds, we should add this to prevent a ``SIGTERM`` from
    being sent to the nodes. If nodes are not terminated correctly they can leave connections open, linger indefinitely,
    and cause extremely difficult-to-debug situations. This can also be dangerous when real robots are used.

.. code-block:: console

    ros2 launch nav2_bringup tb3_simulation_launch.py use_sim_time:=True headless:=False sigterm_timeout:=120

We use ``headless:=False`` for the launch file to spawn :program:`Gazebo`. We use ``use_sim_time:=True`` to make sure
that :program:`Gazebo`\'s clock will be used an prevent timing issues with `/tf`.

This example will create a large number of nodes and open two screens. One of these will be for :program:`Gazebo` and
another for :program:`rviz2`. In this example, there are two actions expected from the user.

#. Set the initial *2D Pose Estimate*. This can be done through :program:`rviz2`, or :program:`ROS2` interfaces.
#. Send one or more *Nav2 Goal* \s.

An example of how to do so is shown in the video below. Please note that although I adjust :program:`Gazebo` to line
up with the :program:`rviz2` view, this is for my convenience. As long as your initial estimate is fairly accurate
the navigation should work fine.

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/eoZ6lu1YWQo?si=vDkECcj_1oQtgHcl" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

.. warning::

    The ``jazzy`` version of this example shows a number of errors when shutting down. This might cause issues
    when the example is run repeatedly.


Unpacking the example
---------------------

Locally, you can navigate to the package using the following command.

.. code-block::

    cd $(ros2 pkg prefix nav2_bringup --share)

The installed folder will have the following structure.

.. code-block:: console

    nav2_bringup/
    ├── cmake
    │   ├── nav2_bringupConfig.cmake
    │   └── nav2_bringupConfig-version.cmake
    ├── environment
    │   ├── ament_prefix_path.dsv
    │   ├── ament_prefix_path.sh
    │   ├── path.dsv
    │   └── path.sh
    ├── launch
    │   ├── bringup_launch.py
    │   ├── cloned_multi_tb3_simulation_launch.py
    │   ├── localization_launch.py
    │   ├── navigation_launch.py
    │   ├── rviz_launch.py
    │   ├── slam_launch.py
    │   ├── tb3_loopback_simulation.launch.py
    │   ├── tb3_simulation_launch.py
    │   ├── tb4_loopback_simulation.launch.py
    │   ├── tb4_simulation_launch.py
    │   └── unique_multi_tb3_simulation_launch.py
    ├── local_setup.bash
    ├── local_setup.dsv
    ├── local_setup.sh
    ├── local_setup.zsh
    ├── maps
    │   ├── depot.pgm
    │   ├── depot.yaml
    │   ├── tb3_sandbox.pgm
    │   ├── tb3_sandbox.yaml
    │   ├── warehouse.pgm
    │   └── warehouse.yaml
    ├── package.dsv
    ├── package.xml
    ├── params
    │   ├── nav2_multirobot_params_1.yaml
    │   ├── nav2_multirobot_params_2.yaml
    │   ├── nav2_multirobot_params_all.yaml
    │   └── nav2_params.yaml
    └── rviz
        ├── nav2_default_view.rviz
        └── nav2_namespaced_view.rviz


The launch file we executed in this example is shown below. A launch file that is general will tend to have a proportional
level of complexity. We are not currently interested in dissecting all elements of this file. Rather, our interest is to
take a look at the relevant files used and what they mean. By understanding these, we would be a step closer to be able to
modify the example or interact with it for your own purposes.

.. dropdown:: Contents of :file:`tb3_simulation_launch.py`

    .. rli:: https://raw.githubusercontent.com/ros-navigation/navigation2/refs/heads/jazzy/nav2_bringup/launch/tb3_simulation_launch.py
        :language: python
        :linenos:
        :emphasize-lines: 92,103,153,163,167

The highlighted lines point out to these important files.

- :file:`maps/tb3_sandbox.yaml`
- :file:`params/nav2_params.yaml`
- :file:`worlds/tb3_sandbox.sdf.xacro`
- :file:`urdf/gz_waffle.sdf.xacro`
- :file:`urdf/turtlebot3_waffle.urdf`

.. rli:: https://raw.githubusercontent.com/ros-navigation/navigation2/refs/heads/jazzy/nav2_bringup/worlds/tb3_sandbox.sdf.xacro
    :language: xml
    :linenos:

.. rli:: https://raw.githubusercontent.com/ros-navigation/navigation2/refs/heads/jazzy/nav2_bringup/urdf/gz_waffle.sdf.xacro
    :language: xml
    :linenos:

.. rli:: https://raw.githubusercontent.com/ros-navigation/navigation2/refs/heads/jazzy/nav2_bringup/urdf/turtlebot3_waffle.urdf
    :language: xml
    :linenos:

Navigation :file:`nav2_params.yaml`
-----------------------------------

.. seealso::

    Official documentation: https://docs.nav2.org/configuration/index.html

In the example, the navigation is defined in the following configuration file.

.. dropdown:: Contents of :file:`nav2_params.yaml`

    .. rli:: https://raw.githubusercontent.com/ros-navigation/navigation2/refs/heads/jazzy/nav2_bringup/params/nav2_params.yaml
        :language: yaml
        :linenos:

Once more, there is a trade-off between generality and the complexity of the configuration file. We can unpack the major
elements of the configuration in the table below.

In ``nav2``, each functionality is divided into a so-called *server*. Each server will have its own set of parameters and
will communicate with other servers. In this topology, ideally one functionality can be modified without affecting
other servers. These are not all servers available in ``nav2``, but all the servers used in this example.

===================== ================================= =============================================================================
Server                TL;DR                             Link
===================== ================================= =============================================================================
``amcl``              Adaptive Monte-Carlo Localiser    `docs <https://docs.nav2.org/configuration/packages/configuring-amcl.html>`_
``bt_navigator``      Behavior tree navigator           `docs <https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html>`_
``controller_server`` Controller server                 `docs <https://docs.nav2.org/configuration/packages/configuring-controller-server.html>`_
``local_costmap``     A 2D costmap                      `docs <https://docs.nav2.org/configuration/packages/configuring-costmaps.html>`_
``global_costmap``    A 2D costmap                      `docs <https://docs.nav2.org/configuration/packages/configuring-costmaps.html>`_
``planner_server``    Calculates path to goal           `docs <https://docs.nav2.org/configuration/packages/configuring-planner-server.html>`_
``smoother_server``   Keeps path smooth                 `docs <https://docs.nav2.org/configuration/packages/configuring-smoother-server.html>`_
``behavior_server``   Defines basic robot behaviors     `docs <https://docs.nav2.org/configuration/packages/configuring-behavior-server.html>`_
``route_server``      Handles pre-defined routes        `docs <https://docs.nav2.org/configuration/packages/configuring-route-server.html>`_
``velocity_smoother`` Smooth velocities sent to robots  `docs <https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html>`_
``collision_monitor`` Extra layer of safety             `docs <https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html>`_
===================== ================================= =============================================================================

Each server can be attached to one or more ``nav2`` *plugins*. The plugins will define important aspects of the
server and plugins will have their own parameters.

.. seealso::
    Official documentation: https://docs.nav2.org/plugins/index.html

In ``nav2``, mainly, there will be a *global* planner (in this case ``nav2_navfn_planner::NavfnPlanner``) and a *local*
controller (in this case ``nav2_mppi_controller::MPPIController``). In general terms, the global planner can find solutions
when the goal is far and create a trajectory. The controller will be responsible for dealing with following the trajectory
in the short term. This split is common in robotics.

map :file:`nav2_params.yaml`
-----------------------------------

.. rli:: https://raw.githubusercontent.com/ros-navigation/navigation2/refs/heads/jazzy/nav2_bringup/maps/tb3_sandbox.yaml
    :language: yaml
    :linenos:

A costmap in ``nav2`` is a 2D grid in which each cell is assigned a value, similarly to an image. Using the image analogy,
each *costmap* (image) will be made of *costs* (pixels). The value in each pixel will define how *costly* it is to move
through that pixel.

Suppose that we have the small costmap below, for illustrative purposes. Suppose that we use ``F`` for free space, U
for unknown, and O for occupied. We can see this as a top-view image of the path that the robot can traverse. The planner
and controller will use this information.

+--+------+------+--+--+
|O |O     |O     |O |U |
+--+------+------+--+--+
|O |``F`` |``F`` |O |U |
+--+------+------+--+--+
|O |O     |``F`` |O |U |
+--+------+------+--+--+
|O |``F`` |``F`` |O |U |
+--+------+------+--+--+
|O |O     |O     |O |U |
+--+------+------+--+--+

Navigation with SLAM
++++++++++++++++++++

.. seealso::

    Official documentation: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html#navigation2-with-slam

.. code-block:: console

    ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True sigterm_timeout:=120