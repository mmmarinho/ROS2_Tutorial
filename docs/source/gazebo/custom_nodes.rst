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

.. important::

    The scene with a :file:`gz::sim::systems::ApplyLinkWrench` will also need to have a ``gz::sim::systems::SceneBroadcaster``
    or the scene will not load.

We will modify our ``shapes_with_tf2_publisher.sdf`` to add the following lines inside the ``<world>`` tag.

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

Add the following file to your :file:`~/gazebo_tutorial_workspace/scenes` folder.

:download:`shapes_with_tf2_and_wrench.sdf <../../../gazebo_tutorial_workspace/scenes/shapes_with_tf2_and_wrench.sdf>`

.. dropdown:: Contents of :file:`shapes_with_tf2_and_wrench.sdf`

    .. literalinclude:: ../../../gazebo_tutorial_workspace/scenes/shapes_with_tf2_and_wrench.sdf
       :language: xml
       :linenos:
       :emphasize-lines: 14-25

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

1. Send :program:`ROS2` messages to the ``[...]/wrench`` :program:`Gazebo` topic.
    - It uses ``gz.msgs.EntityWrench``, therefore the pairing message is ``ros_gz_interfaces/msg/EntityWrench``.
2. Use a :program:`ROS2` service to call the ``[...]/set_pose`` :program:`Gazebo` service.
    - It uses ``gz.msgs.Pose`` for the request, therefore the pairing message can be ``geometry_msgs/msg/Pose``.
    - It uses ``gz.msgs.Boolean`` for the response, therefore the pairing message is ``std_msgs/msg/Bool``.


Create the package
------------------

We start by creating a package that depends on the interface packages mentioned above, namely ``ros_gz_interfaces``,
``geometry_msgs``, and ``std_msgs``.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_gazebo \
    --build-type ament_python \
    --dependencies rclpy ros_gz_interfaces geometry_msgs std_msgs

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