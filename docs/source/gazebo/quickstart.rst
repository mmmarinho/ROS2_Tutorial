.. include:: ../the_topic_is_under_heavy_construction.rst

Using :program:`Gazebo`
=======================

.. hint::

    You can start the simulator with

    .. code-block:: console

        gz sim

In this section we will see the basic operations related to :program:`Gazebo` and create our own :file:`sdf` file.

Basic functionality
-------------------

These two official tutorials cover the basic functionality of :program:`Gazebo`. They are very well made with up-to-date images and videos. Please go through them to familiarise yourself with the basic functionality.

.. hint::

    In the ``Understanding the GUI`` tutorial, you can skip the part about using ``--force-version``.
    The current version of :program:`Gazebo Harmonic` is ``8.9.0`` so the command would be changed to

    .. code-block:: console

        gz sim --force-version 8.9.0 shapes.sdf


    But you should not have multiple versions of :program:`Gazebo` installed anyway!!

- `Understanding the GUI <https://gazebosim.org/docs/harmonic/gui/>`_.
- `Manipulating Models <https://gazebosim.org/docs/harmonic/manipulating_models/>`_

Fun with plugins
----------------

.. note::

   Apply Force Torque Plugin
   https://gazebosim.org/api/sim/8/apply_force_torque.html

   Mouse Drag
   https://gazebosim.org/api/sim/8/mouse_drag.html

We can start by choosing the file :file:`ackermann_steering.sdf`

.. _curl: https://curl.se/
.. _`lsb-release`: https://packages.debian.org/sid/lsb-release
.. _gnupg: https://gnupg.org/download/


:program:`Gazebo` and :program:`ROS2` structure
-----------------------------------------------

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
