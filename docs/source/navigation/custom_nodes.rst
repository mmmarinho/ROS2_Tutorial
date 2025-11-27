Interface ``nav2`` with custom :program:`ROS2` nodes
====================================================

.. versionadded:: Jazzy

   This section.

.. include:: ../the_topic_is_under_heavy_construction.rst


Objective
---------

We are going to create custom code to interact with the navigation stack, ``nav2``. This will be based on the demo
``tb3_simulation_launch.py``, but it is generally applicable. In the example, ``rviz2`` is used for interactivity.
In this example, we will be able to operate ``nav2`` automatically, without relying on the visualizer.

#. Create a publisher to send the initial pose.
#. Create an action client to send navigation goals.

.. include:: ./tb3_simulation_launch_disclaimer.rst

When dealing with a new stack you might not be familiar with, the first step is to try to make sense of the interfaces
available.

Initial pose topic
++++++++++++++++++

When the ``tb3_simulation_launch.py`` example is running, if we run ``ros2 topic list`` you will see a long
list of topics. Let us filter them out with the keyword *pose*, giving that we're trying to find things related to
pose. We can do so with the command below.

.. code-block:: console

    ros2 topic list | grep pose

The output of that will be the following topics.

.. code-block:: console

    /amcl_pose
    /detected_dock_pose
    /dock_pose
    /filtered_dock_pose
    /goal_pose
    /initialpose
    /staging_pose

This is convenient for us because ``/initialpose`` describes exactly what we want to do in the first step. If you are
annoyed by the fact that the words *initial* and *pose* are not separated by a *_*, so am I.

We can check if this is in fact the correct topic used by :program:`rviz2`. With the demo running, we run, in another
terminal, the following command.

.. code-block:: console

    ros2 topic echo /initialpose

Then, we send the initial position through :program:`rviz2`. We can see that in fact :program:`rviz2` used this topic
to set the initial pose, therefore our guess was correct. Below is not representative output.

.. dropdown:: Output of ros2 topic echo

    .. code-block:: console

        header:
          stamp:
            sec: 247
            nanosec: 791000000
          frame_id: map
        pose:
          pose:
            position:
              x: -2.1046903133392334
              y: -0.3082020580768585
              z: 0.0
            orientation:
              x: 0.0
              y: 0.0
              z: 0.0
              w: 1.0
          covariance:
          - 0.25
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.25
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.0
          - 0.06853891909122467
        ---

We can understand more about the topic with the following command.

.. code-block:: console

    ros2 topic info /initialpose

This will show us that the topic uses :file:`geometry_msgs/msg/PoseWithCovarianceStamped`. Thence, all we have to do
is create a subscriber with that message type to the topic ``/initialpose``.

.. code-block:: console

    Type: geometry_msgs/msg/PoseWithCovarianceStamped
    Publisher count: 1
    Subscription count: 1

Although it might look somewhat trivial at this stage, there are many :program:`ROS2` concepts at play so that these
connections can be understood.

Navigate to pose action
+++++++++++++++++++++++

When operating the demo through :program:`rviz2`, you might have noticed that sending navigation goals behaves as an
action. It has a goal, feedback, and a result. With that in mind, we look through the active action, once more filtering
for *pose*. We can do so with the following command.

.. code-block:: console

    ros2 action list | grep pose

This results in the following actions.

.. code-block:: console

    /compute_path_through_poses
    /compute_path_to_pose
    /navigate_through_poses
    /navigate_to_pose

Good sense should indicate that, given that we want to navigate to a pose, the correct topic in question is
``/navigate_to_pose``. We can check more information about it with the following command.

.. code-block:: console

    ros2 action info /navigate_to_pose

The information clearly states that :program:`rviz2` is connected to it, which is a good indicator that we are on the
right path.

.. code-block:: console

    Action: /navigate_to_pose
    Action clients: 5
        /rviz
        /rviz_navigation_dialog_action_client
        /bt_navigator
        /waypoint_follower
        /docking_server
    Action servers: 1
        /bt_navigator

With the following command we see what type of action we have to support.

.. code-block:: console

    ros2 action type /navigate_to_pose

The output of the command will be the following action type.

.. code-block:: console

    nav2_msgs/action/NavigateToPose

Thence, our next step is to see what are the fields that we need to support in our action client. The long way to do
so is with the command we are already aware of.

.. code-block:: console

    ros2 interface show nav2_msgs/action/NavigateToPose

.. dropdown:: Output of ros2 interface show

    .. code-block:: yaml

        #goal definition
        geometry_msgs/PoseStamped pose
            std_msgs/Header header
                builtin_interfaces/Time stamp
                    int32 sec
                    uint32 nanosec
                string frame_id
            Pose pose
                Point position
                    float64 x
                    float64 y
                    float64 z
                Quaternion orientation
                    float64 x 0
                    float64 y 0
                    float64 z 0
                    float64 w 1
        string behavior_tree
        ---
        #result definition

        # Error codes
        # Note: The expected priority order of the errors should match the message order
        uint16 NONE=0

        uint16 error_code
        string error_msg
        ---
        #feedback definition
        geometry_msgs/PoseStamped current_pose
            std_msgs/Header header
                builtin_interfaces/Time stamp
                    int32 sec
                    uint32 nanosec
                string frame_id
            Pose pose
                Point position
                    float64 x
                    float64 y
                    float64 z
                Quaternion orientation
                    float64 x 0
                    float64 y 0
                    float64 z 0
                    float64 w 1
        builtin_interfaces/Duration navigation_time
            int32 sec
            uint32 nanosec
        builtin_interfaces/Duration estimated_time_remaining
            int32 sec
            uint32 nanosec
        int16 number_of_recoveries
        float32 distance_remaining

A more condensed version of the action can be obtained with the following commands.

.. code-block::

    cd $(ros2 pkg prefix nav2_msgs --share)
    cat action/NavigateToPose.action

The output of this command will be the following.

.. code-block:: yaml

    #goal definition
    geometry_msgs/PoseStamped pose
    string behavior_tree
    ---
    #result definition

    # Error codes
    # Note: The expected priority order of the errors should match the message order
    uint16 NONE=0

    uint16 error_code
    string error_msg
    ---
    #feedback definition
    geometry_msgs/PoseStamped current_pose
    builtin_interfaces/Duration navigation_time
    builtin_interfaces/Duration estimated_time_remaining
    int16 number_of_recoveries
    float32 distance_remaining

So what?
++++++++

After going through this investigative process looking through topics and actions, we reached a conclusion of what
must be done in a more concrete sense.

#. Create a publisher to send the initial pose.
    - The message type is ``geometry_msgs/msg/PoseWithCovarianceStamped``.
#. Create an action client to send navigation goals.
    - The action type is ``nav2_msgs/action/NavigateToPose``.

That's it!

Create the package
------------------

We'll start by creating a suitable :program:`ROS2` package. We already now, from the previous investigation, what
package we need to depend on.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_nav2 \
    --build-type ament_python \
    --dependencies rclpy geometry_msgs nav2_msgs

.. dropdown:: ros2 pkg create output

    .. code-block:: console

        going to create a new package
        package name: python_package_that_uses_nav2
        destination directory: ~/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['rclpy', 'geometry_msgs', 'nav2_msgs']
        creating folder ./python_package_that_uses_nav2
        creating ./python_package_that_uses_nav2/package.xml
        creating source folder
        creating folder ./python_package_that_uses_nav2/python_package_that_uses_nav2
        creating ./python_package_that_uses_nav2/setup.py
        creating ./python_package_that_uses_nav2/setup.cfg
        creating folder ./python_package_that_uses_nav2/resource
        creating ./python_package_that_uses_nav2/resource/python_package_that_uses_nav2
        creating ./python_package_that_uses_nav2/python_package_that_uses_nav2/__init__.py
        creating folder ./python_package_that_uses_nav2/test
        creating ./python_package_that_uses_nav2/test/test_copyright.py
        creating ./python_package_that_uses_nav2/test/test_flake8.py
        creating ./python_package_that_uses_nav2/test/test_pep257.py

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

In this section, we will be creating or modifying the following highlighted files.

.. code-block:: console
    :emphasize-lines: 5,6,10

    python_package_that_uses_nav2/
    |-- package.xml
    |-- python_package_that_uses_nav2
    |   |-- __init__.py
    |   |-- nav2_initial_pose_publisher_node.py
    |   `-- nav2_navigate_to_pose_action_client_node.py
    |-- resource
    |   `-- python_package_that_uses_nav2
    |-- setup.cfg
    |-- setup.py
    `-- test
        |-- test_copyright.py
        |-- test_flake8.py
        `-- test_pep257.py

Sending the initial pose to ``nav2``
------------------------------------

Given that this node is a simple publisher, we can jump straight to it.

:download:`nav2_initial_pose_publisher_node.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/python_package_that_uses_nav2/nav2_initial_pose_publisher_node.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/python_package_that_uses_nav2/nav2_initial_pose_publisher_node.py
   :language: python
   :linenos:
   :lines: 24-

There might be two minor novelties in this example. The first one is similar to what we did in ``tf2`` when guaranteeing
that the publisher was connected before attempting to publish for the first time. In this case, because in this demo
:program:`rviz2` is already connected to that topic, we check that there are two subscribers connected before proceeding
with the initialisation.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/python_package_that_uses_nav2/nav2_initial_pose_publisher_node.py
   :language: python
   :lines: 43-48
   :emphasize-lines: 2

The second part that might be unfamiliar is the covariance used in the message. The covariance matrix holds the variance
in the diagonal and off the diagonal any pair-wise variances between different states. All of this to say that the
covariance will be used to say how confident we are in a given field of the pose and if their variation is correlated.
In this case, we choose the same, or similar, values that were sent by :program:`rviz2`. This is enough for our purposes.
The covariance is highlighted below.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/python_package_that_uses_nav2/nav2_initial_pose_publisher_node.py
   :language: python
   :lines: 50-72
   :emphasize-lines: 19-21

It is important not to be distracted by the nested ``pose``. The first one os the ``PoseWithCovariance``, part of the
``PoseWithCovarianceStamped``. The second one is the ``Pose`` part of the ``PoseWithCovariance``.

That's it. This node is relatively simple: a publisher that publishes a single message. There will be other ways to achieve
this. The one shown herein is the most standard one according to what has been shown in the tutorial.

Handling ``nav2`` pose navigation actions
-----------------------------------------

I'm conflicted whether this one is even easier. I suppose it's more complex, being an action client. However, it
follows the same formula as we did in the action tutorial.

:download:`nav2_navigate_to_pose_action_client_node.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/python_package_that_uses_nav2/nav2_navigate_to_pose_action_client_node.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/python_package_that_uses_nav2/nav2_navigate_to_pose_action_client_node.py
   :language: python
   :linenos:
   :lines: 24-

We can highlight two possible pinch points. Starting by the ``main()`` function, we instantiate the node and create
a desired pose, highlighted below. Indeed, we are going to use a stamped pose. However, it is simpler to grab the
stamp from a node, so we do that in a method. We send an empty behaviour tree, because otherwise would be out of the
scope of this tutorial.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/python_package_that_uses_nav2/nav2_navigate_to_pose_action_client_node.py
   :language: python
   :lines: 87-94
   :emphasize-lines: 3-6

Then, in our implementation we populate the fields of the goal with the pose we just created and the stamp obtained
from the node. We receive an empty behaviour tree and that is also added to the goal.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/python_package_that_uses_nav2/nav2_navigate_to_pose_action_client_node.py
   :language: python
   :lines: 44-49

Besides the slightly different action type, the process to make an action client is mostly unchanged. 

Adjusting the :file:`setup.py`
------------------------------

:download:`setup.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/setup.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nav2/setup.py
   :language: python
   :emphasize-lines: 27-28
   :linenos:

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: ../the_canonical_build_command.rst

Testing
-------

The first step is to run the same demo as before, with ``tb3_simulation_launch.py``. We can do that with the command
below.

.. code-block:: console

    ros2 launch nav2_bringup \
    tb3_simulation_launch.py \
    use_sim_time:=True \
    headless:=False \
    sigterm_timeout:=120

Then, in another terminal, we can run the following command. This one will be rather tolerant of what it's started
because it will wait for the correct number of publishers before publishing.

.. code-block:: console

    ros2 run python_package_that_uses_nav2 nav2_initial_pose_publisher_node

The following command should only be attempted after the initial pose was set. The navigation stack will not accept
goals unless an initial estimate is given first.

.. code-block:: console

    ros2 run python_package_that_uses_nav2 nav2_navigate_to_pose_action_client_node

The qualitative behaviour can be seen in the following video.

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/yZUyu0iefxA?si=9p3Fq1th3a8Uv-5v" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

.. admonition:: Exercises

    Other possible interactions with these interfaces could be as follows.

    - What would change if the same node was supposed to both send the initial pose and, after that is done, send
      the goal?
    - What would have to be modified if you had three goals, goal ``a``, goal ``b``, and goal ``c``. Suppose that we
      initially send goal ``a``. If the goal is reached, then go to goal ``b``. If not reached, go to goal ``c``. This
      is a very basic state machine. An embryo, but somewhat the motivation for behaviour tress.

