Interface :program:`Gazebo` with custom :program:`ROS2` nodes
=============================================================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

.. seealso::

    This would be the official way: https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/

In this section, our intention is to control things in :program:`Gazebo` from a :program:`ROS2` package, using nodes
and launch files as needed. For things that are well supported in :program:`Gazebo`, this will work fine. For anything
new, you might need to dive more deeply into :program:`Gazebo` plugins, which is considerably outside the scope of a
tutorial like this one.

There is a lot of sample code available online for :program:`Gazebo` integration. You will also see frequent patterns of
having multiple packages in a single project, to correctly organize the package according to :program:`Gazebo` logic.
This logic tends to be reflected in other packages, such as examples using ``nav2``.

Most of the sample code I found relates to the use of :program:`Gazebo` plugins with specific functionality,
such as ``gz::sim::systems::DiffDrive``, ``gz::sim::systems::JointStatePublisher``, and ``gz::sim::systems::OdometryPublisher``.

Instead of repeating any of that, the idea here is to show how a simple node can perform simple communication with
:program:`Gazebo`. Using this logic, you can find out how to communicate with any existing plugins, updated plugins, or
new plugins when they become available.

.. note::

    I haven't yet found a comprehensive list of :program:`Gazebo` plugins and their pairing ``xml`` descriptions to add them to a ``.sdf``.
    Most of the information is spread around examples and takes a bit a trial-and-error to find dependencies between
    plugins as error messages and warnings are not currently output by default.

    Please feel free to correct me `here <https://github.com/mmmarinho/ROS2_Tutorial/issues/new>`_.

Setting up the scene
--------------------

We will use a slightly modified version of the :file:`shapes_with_tf2_publisher.sdf` created previously. Let's add
it to our :program:`Gazebo` folder.

We will use two systems in this example. We will use ``gz::sim::systems::UserCommands`` to allow us to set the pose of
entities and ``gz::sim::systems::ApplyLinkWrench`` to allow us to set wrenches to links. This is illustrative because
poses use :program:`Gazebo` services and wrenches use :program:`Gazebo` topics.

We will modify our ``shapes_with_tf2_publisher.sdf`` to add the following lines inside the ``<world>`` tag.

.. literalinclude:: ../../../gazebo_tutorial_workspace/scenes/shapes_with_tf2_and_wrench.sdf
   :language: xml
   :lines: 13-21

Here's a brief explanation of why we need each one of these.

.. note::

    More information is available in the :file:`gz-sim/src/systems` folder at https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/systems.

+---------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
|``gz::sim::systems::Physics``          |  The physics behavior needed by the other systems.                                                                                         |
+---------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
|``gz::sim::systems::SceneBroadcaster`` |  Needed by the other systems `[doc] <https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/scene_broadcaster/SceneBroadcaster.hh>`_.|
+---------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
|``gz::sim::systems::ApplyLinkWrench``  |  Creates the ``/wrench`` topic `[doc] <https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/user_commands/UserCommands.hh>`_.      |
+---------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+
|``gz::sim::systems::UserCommands``     |  Creates the ``/set_pose`` topic `[doc] <https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/user_commands/UserCommands.hh>`_.    |
+---------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------+

We start by adding the following file to your :file:`~/gazebo_tutorial_workspace/scenes` folder.

:download:`shapes_with_tf2_and_wrench.sdf <../../../gazebo_tutorial_workspace/scenes/shapes_with_tf2_and_wrench.sdf>`

.. dropdown:: Contents of :file:`shapes_with_tf2_and_wrench.sdf`

    .. literalinclude:: ../../../gazebo_tutorial_workspace/scenes/shapes_with_tf2_and_wrench.sdf
       :language: xml
       :linenos:
       :emphasize-lines: 13-21

We can open this scene in :program:`Gazebo` with the following command.

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

The message used in this topic can be found with the following command.

.. code-block:: console

    gz topic -i -t /world/shapes_with_tf2_and_wrench/wrench

It should result in the following output.

.. code-block:: console

    No publishers on topic [/world/shapes_with_tf2_and_wrench/wrench]
    Subscribers [Address, Message Type]:
      tcp://172.16.191.128:37021, gz.msgs.EntityWrench

Using the following command will show the pose-related services.

.. code-block:: console

    gz service -l | grep set_pose

It should result in the following output.

.. code-block:: console

    /world/shapes_with_tf2_and_wrench/set_pose
    /world/shapes_with_tf2_and_wrench/set_pose/blocking
    /world/shapes_with_tf2_and_wrench/set_pose_vector
    /world/shapes_with_tf2_and_wrench/set_pose_vector/blocking

The request and response of this service can be found with the following command.

.. code-block:: console

    gz service -i -s /world/shapes_with_tf2_and_wrench/set_pose

It should result in the following output.

.. code-block:: console

    Service providers [Address, Request Message Type, Response Message Type]:
      tcp://172.16.191.128:35577, gz.msgs.Pose, gz.msgs.Boolean


Objective
---------

Our objectives in this section will be as follows.

1. Send :program:`ROS2` messages to the ``[...]/wrench`` :program:`Gazebo` topic. It uses ``gz.msgs.EntityWrench``,
   therefore the pairing message is ``ros_gz_interfaces/msg/EntityWrench``.
2. Use a :program:`ROS2` service to call the ``[...]/set_pose`` :program:`Gazebo` service. The interface is not listed in
   the official list, but we will use ``ros_gz_interfaces/srv/SetEntityPose``. See the official
   repository: https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_interfaces/srv/SetEntityPose.srv.

The pose messages will be processed by ``tf2``.

Create the package
------------------

We start by creating a package that depends on the interface packages mentioned above, namely ``ros_gz_interfaces`` and
``tf2_ros``.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_gazebo \
    --build-type ament_python \
    --dependencies rclpy ros_gz_interfaces tf2_ros

.. dropdown:: ros2 pkg create output

   .. code :: console

        going to create a new package
        package name: python_package_that_uses_gazebo
        destination directory: /IdeaProjects/ROS2_Tutorial/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['rclpy', 'ros_gz_interfaces', 'geometry_msgs', 'std_msgs']
        creating folder ./python_package_that_uses_gazebo
        creating ./python_package_that_uses_gazebo/package.xml
        creating source folder
        creating folder ./python_package_that_uses_gazebo/python_package_that_uses_gazebo
        creating ./python_package_that_uses_gazebo/setup.py
        creating ./python_package_that_uses_gazebo/setup.cfg
        creating folder ./python_package_that_uses_gazebo/resource
        creating ./python_package_that_uses_gazebo/resource/python_package_that_uses_gazebo
        creating ./python_package_that_uses_gazebo/python_package_that_uses_gazebo/__init__.py
        creating folder ./python_package_that_uses_gazebo/test
        creating ./python_package_that_uses_gazebo/test/test_copyright.py
        creating ./python_package_that_uses_gazebo/test/test_flake8.py
        creating ./python_package_that_uses_gazebo/test/test_pep257.py

        [WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
        It is recommended to use one of the ament license identifiers:
        Apache-2.0
        BSL-1.0
        BSD-2.0
        BSD-2-Clause
        BSD-3-Clause
        GPL-3.0-only
        LGPL-3.0-only
        MIT
        MIT-0

Files
-----

We will be working on the following files.

.. code-block:: console
    :emphasize-lines: 3,5-7,11-13,17

    python_package_that_uses_gazebo/
    |-- config_bridge
    |   `-- control_shape_thrust.yaml
    |-- launch
    |   |-- control_shape_thrust_launch.py
    |   |-- send_poses_to_gazebo_launch.py
    |   `-- send_wrenches_to_gazebo_launch.py
    |-- package.xml
    |-- python_package_that_uses_gazebo
    |   |-- __init__.py
    |   |-- control_shape_thrust_node.py
    |   |-- send_poses_to_gazebo_node.py
    |   `-- send_wrenches_to_gazebo_node.py
    |-- resource
    |   `-- python_package_that_uses_gazebo
    |-- setup.cfg
    |-- setup.py
    `-- test
        |-- test_copyright.py
        |-- test_flake8.py
        `-- test_pep257.py

Sending poses to :program:`Gazebo`
----------------------------------

.. admonition:: Summary

    We will make a node that does the following for us through :program:`ROS2`.

    .. code-block:: console

            gz service -s \
            /world/shapes_with_tf2_and_wrench/set_pose \
            --reqtype gz.msgs.Pose \
            --reptype gz.msgs.Boolean \
            --req 'name: "box", position: {x: 0.0, y: 0.0, z: 50}'

We'll be able to send poses to :program:`Gazebo` with the following node. It is a relatively simple node with a
service client.

:download:`send_poses_to_gazebo_node.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/send_poses_to_gazebo_node.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/send_poses_to_gazebo_node.py
   :language: python
   :lines: 24-
   :linenos:

There are perhaps only two unfamiliar aspects of this node by now. First, that we need to send the correct entity
name for :program:`Gazebo` to know which entity to set the pose. This name can be obtained in :program:`Gazebo`\`s GUI.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/send_poses_to_gazebo_node.py
   :language: python
   :lines: 44-63
   :emphasize-lines: 7

The second possibly unfamiliar is the choice of ``rclpy.spin_until_future_complete`` to illustrate calling the service
client only once.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/send_poses_to_gazebo_node.py
   :language: python
   :lines: 66-81
   :emphasize-lines: 11

.. admonition:: Exercises

    How would you approach this node if you had to, for instance.

    - Accept the entity information as a parameter for the node.
    - Accept the pose as a parameter from the node.
    - Set the pose of multiple entities.

The launch file
+++++++++++++++

This node will not work without a pairing ``parameter_bridge``. We add the following launch file to the :file:`launch`
folder.

:download:`send_poses_to_gazebo_launch.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/send_poses_to_gazebo_node.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/launch/send_poses_to_gazebo_launch.py
   :language: python
   :linenos:

Sending wrenches to :program:`Gazebo`
-------------------------------------

.. admonition:: Summary

    We will make a node that does the following for us through :program:`ROS2`.

    .. code-block:: console

            gz topic -t \
            /world/shapes_with_tf2_and_wrench/wrench \
            -m gz.msgs.EntityWrench \
            -p  'entity: {id: 9}, wrench: {force: {x: 1000.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'

We'll be able to send poses to :program:`Gazebo` with the following node. It is a relatively simple node with a
publisher.

:download:`send_wrenches_to_gazebo_node.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/send_wrenches_to_gazebo_node.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/send_wrenches_to_gazebo_node.py
   :language: python
   :lines: 24-
   :linenos:

The only unfamiliar aspect is shown below. The ``id`` of the :program:`Gazebo` entity must be sent. You can obtain
that information from :program:`Gazebo`\'s GUI. Note that here, trying to send a ``name`` instead is inconsequential
and :program:`Gazebo` won't recognise it.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/send_wrenches_to_gazebo_node.py
   :language: python
   :lines: 45-69
   :emphasize-lines: 7

.. admonition:: Wait... what?

    Yes, for wrenches, we need the ``id``. For the pose, we use the ``name``. Such is life.

.. admonition:: Exercises

    How would you approach this node if you had to, for instance.

    - Accept the entity information as a parameter for the node.
    - Accept the wrench as a parameter from the node.
    - Publish the wrench of multiple entities.
    - Define the wrench based on other information?
        - The next section shows one example of it.

The launch file
+++++++++++++++

This node will not work without a pairing ``parameter_bridge``. We add the following launch file to the :file:`launch`
folder.

:download:`send_wrenches_to_gazebo_launch.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/send_wrenches_to_gazebo_launch.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/launch/send_wrenches_to_gazebo_launch.py
   :language: python
   :linenos:

Controlling thrust of shapes
----------------------------

Lastly, we can have a slightly more complex example. In this example, we read the poses through ``tf2``. Then, using
that pose information, we publish a wrench. This way, we can attempt to move a shape to a given place in the scene.

This example highlights one possible way of interacting with :program:`Gazebo`.

:download:`send_poses_to_gazebo_node.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/control_shape_thrust_node.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/control_shape_thrust_node.py
   :language: python
   :lines: 24-
   :linenos:

We start by having multiple hard-coded values to configure the node. This is illustrative and because of how this is
organised it would be easier to turn them into configurable parameters.

As you have seen in previous examples, ``_gazebo_world_name`` is used to define the topic name for the wrench and
also the parent reference frame for ``tf2``. The ``_gazebo_entity_name`` is used to get the child frame for ``tf2``.
Lastly, `_gazebo_entity_id` is used to send the wrench to the correct entity.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/control_shape_thrust_node.py
   :language: python
   :lines: 39-43

There is also a simple proportional controller in one dimension.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/control_shape_thrust_node.py
   :language: python
   :lines: 67-70

Lastly, in the ``timer_callback``, we use the current coordinate value to compute the control action and send it to
:program:`Gazebo`.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/control_shape_thrust_node.py
   :language: python
   :emphasize-lines: 11,13
   :lines: 92-113

.. admonition:: Exercises

    How would you approach this node if you had to, for instance.

    - Accept relevant information as a parameter for the node.
    - Control the shape via thrusts in the 2D plane.
    - Add derivative and integral control actions.
    - Control the shape in 3D.
        - How would the gravity be counterbalanced?

The bridge file
+++++++++++++++

The bridge file, added to the folder :file:`config_bridge`, will manage bridging the topics with ``tf2`` and the wrench.

:download:`send_poses_to_gazebo_launch.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/config_bridge/control_shape_thrust.yaml>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/config_bridge/control_shape_thrust.yaml
   :language: yaml
   :linenos:

The launch file
+++++++++++++++

This node will not work without a pairing ``parameter_bridge``. We add the following launch file to the :file:`launch`
folder.

:download:`send_poses_to_gazebo_launch.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/python_package_that_uses_gazebo/control_shape_thrust_launch.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/launch/control_shape_thrust_launch.py
   :language: python
   :linenos:


Adjusting the :file:`setup.py`
------------------------------

This file will include the directives for all the three nodes, added in ``entry_points``. We also have on L15
the directive for the launch files. Lastly, on L16, we have a directive to install the bridge configuration file
which is used by :file:`control_shape_thrust_launch.py`.

:download:`setup.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/setup.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_gazebo/setup.py
   :language: python
   :emphasize-lines: 1,2,15,16,27-29
   :linenos:

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: ../the_canonical_build_command.rst

Testing
-------

We first open and run the simulation.

.. code-block:: console

    gz sim $HOME/gazebo_tutorial_workspace/scenes/shapes_with_tf2_and_wrench.sdf

It is possible to run each of these examples in this particular order and see their effects.

We can run the first example with the following command. Then,

.. code-block:: console

    ros2 launch python_package_that_uses_gazebo send_poses_to_gazebo_launch.py

.. dropdown:: Output for :file:`send_poses_to_gazebo_launch.py`

    .. code-block::

        [INFO] [launch]: All log files can be found below /home/murilo/.ros/log/2025-11-12-10-29-03-415524-murilo-VMware20-1-6420
        [INFO] [launch]: Default logging verbosity is set to INFO
        [INFO] [send_poses_to_gazebo_node-1]: process started with pid [6423]
        [INFO] [parameter_bridge-2]: process started with pid [6424]
        [parameter_bridge-2] [INFO] [1762943343.652578097] [ros_gz_bridge]: Creating ROS->GZ service bridge [/world/shapes_with_tf2_and_wrench/set_pose (ros_gz_interfaces/srv/SetEntityPose -> /)]
        [INFO] [send_poses_to_gazebo_node-1]: process has finished cleanly [pid 6423]
        ^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
        [parameter_bridge-2] [INFO] [1762943347.930139121] [rclcpp]: signal_handler(signum=2)
        [INFO] [parameter_bridge-2]: process has finished cleanly [pid 6424]


We can run the second example with the following command.

.. code-block:: console

    ros2 launch python_package_that_uses_gazebo send_wrenches_to_gazebo_launch.py

.. dropdown:: Output for :file:`send_wrenches_to_gazebo_launch.py`

    .. code-block::

        [INFO] [launch]: All log files can be found below /home/murilo/.ros/log/2025-11-12-10-29-35-514265-murilo-VMware20-1-6504
        [INFO] [launch]: Default logging verbosity is set to INFO
        [INFO] [send_wrenches_to_gazebo_node-1]: process started with pid [6507]
        [INFO] [parameter_bridge-2]: process started with pid [6508]
        [parameter_bridge-2] [INFO] [1762943375.599973888] [ros_gz_bridge]: Creating ROS->GZ Bridge: [/world/shapes_with_tf2_and_wrench/wrench (ros_gz_interfaces/msg/EntityWrench) -> /world/shapes_with_tf2_and_wrench/wrench (gz.msgs.EntityWrench)] (Lazy 0)
        [send_wrenches_to_gazebo_node-1] Waiting for subscriber to be connected...
        [send_wrenches_to_gazebo_node-1] [INFO] [1762943376.818068638] [send_wrenches_to_gazebo_node]: This sent entity  a wrench with force: geometry_msgs.msg.Vector3(x=1000.0, y=0.0, z=0.0) and torque: geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0).
        [parameter_bridge-2] [INFO] [1762943376.824228506] [ros_gz_bridge]: Passing message from ROS ros_gz_interfaces/msg/EntityWrench to Gazebo gz.msgs.EntityWrench (showing msg only once per type)
        ^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
        [parameter_bridge-2] [INFO] [1762943378.394190781] [rclcpp]: signal_handler(signum=2)
        [INFO] [parameter_bridge-2]: process has finished cleanly [pid 6508]
        [INFO] [send_wrenches_to_gazebo_node-1]: process has finished cleanly [pid 6507]


We can run the third example with the following command.


.. code-block:: console

    ros2 launch python_package_that_uses_gazebo control_shape_thrust_launch.py

.. dropdown:: Output for :file:`control_shape_thrust_launch.py`

    .. code-block::

        [INFO] [launch]: All log files can be found below /home/murilo/.ros/log/2025-11-12-10-30-17-199931-murilo-VMware20-1-6604
        [INFO] [launch]: Default logging verbosity is set to INFO
        /home/murilo/git_another/ROS2_Tutorial/ros2_tutorial_workspace/install/python_package_that_uses_gazebo/share/python_package_that_uses_gazebo/config_bridge/control_shape_thrust.yaml
        [INFO] [control_shape_thrust_node-1]: process started with pid [6607]
        [INFO] [parameter_bridge-2]: process started with pid [6608]
        [control_shape_thrust_node-1] Waiting for subscriber to be connected to /world/shapes_with_tf2_and_wrench/wrench...
        [parameter_bridge-2] [INFO] [1762943418.299653851] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)] (Lazy 0)
        [parameter_bridge-2] [INFO] [1762943418.304154168] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/model/box/pose (gz.msgs.Pose_V) -> /tf (tf2_msgs/msg/TFMessage)] (Lazy 0)
        [parameter_bridge-2] [INFO] [1762943418.316157126] [ros_gz_bridge]: Creating ROS->GZ Bridge: [/world/shapes_with_tf2_and_wrench/wrench (ros_gz_interfaces/msg/EntityWrench) -> /world/shapes_with_tf2_and_wrench/wrench (gz.msgs.EntityWrench)] (Lazy 0)
        [parameter_bridge-2] [INFO] [1762943418.596583200] [ros_gz_bridge]: Passing message from ROS ros_gz_interfaces/msg/EntityWrench to Gazebo gz.msgs.EntityWrench (showing msg only once per type)
        ^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
        [parameter_bridge-2] [INFO] [1762943424.457110149] [rclcpp]: signal_handler(signum=2)
        [INFO] [parameter_bridge-2]: process has finished cleanly [pid 6608]
        [INFO] [control_shape_thrust_node-1]: process has finished cleanly [pid 6607]
