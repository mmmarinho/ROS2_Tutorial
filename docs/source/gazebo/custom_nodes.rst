Interface :program:`Gazebo` with custom :program:`ROS2` nodes
=============================================================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

.. seealso::

    This would be the official way: https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/

In this section, our intention is to control things in :program:`Gazebo` from a :program:`ROS2` package, using nodes
and launch files as needed.

There is a lot of sample code available online for :program:`Gazebo` integration. You will also see frequent patterns of
having multiple packages in a single project, to correctly organize the package according to :program:`Gazebo` logic.
This logic tends to be reflected in other packages, such as examples using ``nav2``.

Most of the sample code I found relates to the use of :program:`Gazebo` plugins with specific functionality,
such as ``gz::sim::systems::DiffDrive``, ``gz::sim::systems::JointStatePublisher``, and ``gz::sim::systems::OdometryPublisher``.

Instead of repeating any of that, the idea here is to show how a simple node can perform simple communication with
:program:`Gazebo`. Using this logic, you can find out how to communicate with any existing plugins, updated plugins, or
new plugins when they become available.

Setting up the scene
--------------------

We will use a slightly modified version of the :file:`shapes_with_tf2_publisher.sdf` created previously. Let's add
it to our :program:`Gazebo` folder.

We will use two systems in this example. We will use ``gz::sim::systems::UserCommands`` to allow us to set the pose of
entities and ``gz::sim::systems::ApplyLinkWrench`` to allow us to set wrenches to links. This is illustrative because
poses use :program:`Gazebo` services and wrenches use :program:`Gazebo` topics.

.. important::

    The scene with a :file:`gz::sim::systems::ApplyLinkWrench` will also need to have a ``gz::sim::systems::SceneBroadcaster``
    or the scene will not load.

.. code-block:: xml

    <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
            filename="gz-sim-apply-link-wrench-system"
            name="gz::sim::systems::ApplyLinkWrench">
    </plugin>
    <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
    </plugin>

.. code-block:: console

    gz sim $HOME/gazebo_tutorial_workspace/scenes/shapes_with_tf2_and_wrench.sdf

Using the following command will show the wrench-related topics.

.. code-block:: console

    gz topic -l | grep /wrench

It should result in the following output.

.. code-block:: console

    /world/shapes_with_tf2_and_wrench/wrench
    /world/shapes_with_tf2_and_wrench/wrench/clear
    /world/shapes_with_tf2_and_wrench/wrench/persistent

Using the following command will show the pose-related services.

.. code-block:: console

    gz service -l | grep set_pose

It should result in the following output.

.. code-block:: console

    /world/shapes_with_tf2_and_wrench/set_pose
    /world/shapes_with_tf2_and_wrench/set_pose/blocking
    /world/shapes_with_tf2_and_wrench/set_pose_vector
    /world/shapes_with_tf2_and_wrench/set_pose_vector/blocking



Create the package
------------------

We start by creating a package to use the action we first created in :ref:`The action file`.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_the_actions \
    --build-type ament_python \
    --dependencies rclpy ros_gz_interfaces

.. dropdown:: ros2 pkg create output

   .. code :: console

        asd

Files
-----

Nodes that interface with :program:`Gazebo`
-------------------------------------------

Adjusting the :file:`setup.py`
------------------------------

This file will include the directives for the server and client. The one for the server is highlighted below.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/setup.py
   :language: python
   :emphasize-lines: 23

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: the_canonical_build_command.rst

Testing
-------