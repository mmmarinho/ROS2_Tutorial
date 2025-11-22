Installation
============

.. versionadded:: Jazzy

   This section.

.. include:: ../the_topic_is_under_heavy_construction.rst

.. seealso::

    Official documentation: https://docs.nav2.org/getting_started/index.html

.. rli:: https://raw.githubusercontent.com/UoMMScRobotics/SFR_Gazebo/refs/heads/main/install_nav2.sh
   :language: bash
   :lines: 5-14

The package ``ros-jazzy-navigation2`` contains most of ``nav2``. However, we have to install ``ros-jazzy-nav2-bringup``
separately owing to "recursive dependencies". See below.

.. dropdown:: Contents of ``ros-jazzy-navigation2``

    .. rli:: https://raw.githubusercontent.com/ros-navigation/navigation2/refs/heads/jazzy/navigation2/package.xml
       :language: xml
       :lines: 15-48

The last package related to ``nav2``, ``ros-jazzy-nav2-minimal-tb*`` has a wildcard that will expand, currently,
to install the following packages.

    - ros-jazzy-nav2-minimal-tb3-sim - Nav2 Minimum TurtleBot3 Simulation
    - ros-jazzy-nav2-minimal-tb4-description - Nav2's minimum Turtlebot4 Description package
    - ros-jazzy-nav2-minimal-tb4-sim - Nav2 Minimum TurtleBot4 Simulation

For illustrative purposes, we also add ``ros-jazzy-slam-toolbox``. Please note that navigation is complementary to
localisation and mapping, but both can be done in isolation too.

.. admonition:: References

    - https://github.com/ros-navigation/navigation2/tree/jazzy
    - https://github.com/ros-navigation/navigation2/tree/jazzy/navigation2
    - https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation/tree/jazzy
    - https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup
    - https://github.com/SteveMacenski/slam_toolbox/tree/jazzy
    - https://docs.nav2.org/concepts/index.html
    - https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
    - https://docs.nav2.org/setup_guides/sdf/setup_sdf.html
    - https://docs.nav2.org/setup_guides/odom/setup_odom_gz.html