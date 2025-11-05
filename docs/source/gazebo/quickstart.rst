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
    The current version of :program:`Gazebo Harmonic` is ``8.9.0`` so the command would be changed to

    .. code-block:: console

        gz sim --force-version 8.9.0 shapes.sdf


    But you should not have multiple versions of :program:`Gazebo` installed anyway!

These two official tutorials cover the basic functionality of :program:`Gazebo`.
They are very well made with up-to-date images and videos. Please go through them to familiarise yourself with the basic functionality.

- `Understanding the GUI <https://gazebosim.org/docs/harmonic/gui/>`_.
- `Manipulating Models <https://gazebosim.org/docs/harmonic/manipulating_models/>`_.

Fun with plugins
----------------

Check the guides below for basic functionality. I advise starting by choosing the file :file:`ackermann_steering.sdf` in these plugins.

- `Apply force torque plugin <https://gazebosim.org/api/sim/8/apply_force_torque.html>`_.
- `Mouse drag plugin <https://gazebosim.org/api/sim/8/mouse_drag.html>`_.

Gazebo from ROS2
----------------

.. code-block:: console

    ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=shapes.sdf

In this command, ``gz_args`` we add a representation file that exists within the Gazebo paths. In this case,
``empty.sdf``.

Using ``ros_gz_bridge``
-----------------------

Spawn models
++++++++++++

.. code-block:: console

    ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5

References
----------

    The official documentation for ``gazebo`` is very good. Here are some main topics where this tutorial
    borrowed from.


    - https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/
    - https://gazebosim.org/docs/harmonic/ros2_integration/
    - https://gazebosim.org/docs/harmonic/ros2_spawn_model/
    - https://gazebosim.org/docs/harmonic/ros2_interop/
    - https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/

