Using ``ros_gz_bridge``
=======================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

.. seealso::

    Official documentation: https://docs.ros.org/en/jazzy/p/ros_gz_bridge/

We have seen how to see internal :program:`Gazebo` topics through the command line. This is not yet very useful to us,
because we want to be able to integrate :program:`Gazebo` with our own, custom :program:`ROS2` nodes.

The idea is to map :program:`Gazebo` topics into :program:`ROS2` topics. This is where ``ros_gz_bridge`` comes in.

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
-------

.. seealso::

    Official documentation: https://gazebosim.org/docs/harmonic/sensors/

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
+++++++++++++

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
+++++++++++++++++

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

Getting Pose Information
------------------------

One important aspect of entities in any simulation is their pose. We must be able to obtain poses to define the behavior
of our robots. For instance, if a box in a simulation moves, the controller might need this information to define
evasive maneuvers.

The most basic manner to be able to obtain the pose of an entity in :program:`Gazebo` from an external program is to add
the following to each model, in the :file:`.sdf` scene.

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

Getting transforms
------------------

.. warning::

    You'll notice that each scene creates a :program:`Gazebo` topic called :file:`/world/<SCENE>/pose/info` where
    :file:`<SCENE>` is the name of the active scene. Although this publishes useful information when checking through
    :program:`Gazebo` topics, it does not play well with :program:`ros_gz_bridge`. You can get the transform information
    without the headers, which in :program:`ROS2` is not useful.

    .. seealso::

        https://github.com/gazebosim/ros_gz/issues/172

Integration with other parts of :program:`ROS2` is important, therefore it could be useful to have all poses of relevant
objects available for ``tf2``. In this case, we need to modify the message that is published by :program:`Gazebo`, but
otherwise there is no big difference.

Our use of ``gz::sim::systems::PosePublisher`` will be slightly modified to have ``<use_pose_vector_msg>true</use_pose_vector_msg>``,
highlighted below.

.. code-block:: xml
    :emphasize-lines: 4

    <plugin
            filename="gz-sim-pose-publisher-system"
            name="gz::sim::systems::PosePublisher">
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <publish_link_pose>false</publish_link_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose>
    </plugin>

Add the following file to your :file:`~/gazebo_tutorial_workspace/scenes` folder.

:download:`shapes_with_tf2_publisher.sdf <../../../gazebo_tutorial_workspace/scenes/shapes_with_tf2_publisher.sdf>`

.. dropdown:: Contents of :file:`shapes_with_tf2_publisher.sdf`

    This is :file:`shapes_with_pose_publisher.sdf` with one additional line in each ``gz::sim::systems::PosePublisher``
    plugin.

    .. literalinclude:: ../../../gazebo_tutorial_workspace/scenes/shapes_with_tf2_publisher.sdf
       :language: xml
       :linenos:
       :emphasize-lines: 98,148,196,245,292,334

We open :program:`Gazebo` with this scene, as follows, then run the simulation by clicking the run button.

.. code-block:: console

    gz sim ~/gazebo_tutorial_workspace/scenes/shapes_with_tf2_publisher.sdf

.. note::

    Don't forget to run the simulation otherwise most information will not be available.

In another terminal, we can check the type of message being used to publish poses.

.. code-block:: console

    gz topic -i --topic /model/box/pose

The output will be similar to the one below. Our modification in the plugin makes the :program:`Gazebo` topic use a different
message type, ``gz.msgs.Pose_V``.

.. code-block:: console

    Publishers [Address, Message Type]:
      tcp://172.16.191.128:34555, gz.msgs.Pose_V
    Subscribers [Address, Message Type]:
      tcp://172.16.191.128:37389, gz.msgs.Pose_V

The idea now is to bridge that type of message with ``tf2``. Beyond simply bridging the topics, we have to make sure that
these two points are correct.

- All transforms are published to a topic named ``/tf``, for compatibility with ``tf2``.
- The clock used in :program:`ROS2` matches the one used in :program:`Gazebo`. This will make sure that when we look
  up transforms the timestamp will make sense.

Now, because we have seven topics to bridge, it would not be practical to have that done through the command line. In addition
we need topics that have different names in :program:`Gazebo` and :program:`ROS2`.

We can take advantage of :file:`.yaml` configuration files that are supported by :program:`ros_gz_bridge`.

To show how this works, let us create a folder for our :file:`.yaml` bridge files.

.. code-block:: console

    mkdir -p ~/gazebo_tutorial_workspace/bridge_config
    cd ~/gazebo_tutorial_workspace/bridge_config

We add the following file to the newly created folder. The ``/clock`` bridge and the ``/tf`` bridge elements that are new
are highlighted.

:download:`transforms.yaml <../../../gazebo_tutorial_workspace/bridge_config/transforms.yaml>`

.. literalinclude:: ../../../gazebo_tutorial_workspace/bridge_config/transforms.yaml
   :language: yaml
   :emphasize-lines: 1-5,7,13,19,25,31,37
   :linenos:

We now have to call :program:`ros_gz_bridge` using this newly created file. Note that some :program:`bash` commands will
not expand ``~`` into the home folder. We can replace those instances safely with ``$HOME``. We send any configuration we
want with the flag ``--ros-args -p``, then set the parameter ``config_file`` to have a path to our newly created configuration
file.

.. code-block:: console
    :emphasize-lines: 3,4

    ros2 run ros_gz_bridge \
    parameter_bridge \
    --ros-args -p \
    config_file:=$HOME/gazebo_tutorial_workspace/bridge_config/transforms.yaml

In the messages published to ``tf2`` by gazebo, you will notice that the frame of reference is the name of the scene.
In this case, ``shapes_with_pose_publisher``. Therefore, to see that the frames are correctly published via :program:`ROS2`,
we can see them in :program:`rviz2`.

.. code-block:: console

    ros2 run rviz2 rviz2 -f shapes_with_pose_publisher

After adding the ``TF`` display, you'll be able to see all the relevant frames.

.. admonition:: Exercise

    Suppose that you added a new model to :file:`shapes_with_tf2_publisher.sdf` and you wanted to publish that information
    to ``\tf`` as well. What files would you modify and what steps would you take to make that possible?

