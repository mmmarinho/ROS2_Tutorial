Using :file:`tf2`
=================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. hint::

    An official ``tf2`` user guide is available at

    - https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html.

Perhaps the most important benefit of ``tf2`` is how it connects with other :program:`ROS2` parts, such as ``rviz2``.
It is a common language used across well-established :program:`ROS2` packages.

The usage is not too far from what you have learned so far, but there a few aspects that differ.

Create the package
------------------

.. note::

    The package is called ``tf2_ros`` in :program:`ROS2`.


We start by creating the :file:`python_package_that_uses_tf2` package. Please note that it must depend on ``tf2_ros``.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_tf2 \
    --build-type ament_python \
    --dependencies geometry_msgs tf2_ros

.. dropdown:: ros2 pkg create output

    .. code-block:: console

        going to create a new package
        package name: python_package_that_uses_tf2
        destination directory: ~/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['geometry_msgs', 'tf2_ros']
        creating folder ./python_package_that_uses_tf2
        creating ./python_package_that_uses_tf2/package.xml
        creating source folder
        creating folder ./python_package_that_uses_tf2/python_package_that_uses_tf2
        creating ./python_package_that_uses_tf2/setup.py
        creating ./python_package_that_uses_tf2/setup.cfg
        creating folder ./python_package_that_uses_tf2/resource
        creating ./python_package_that_uses_tf2/resource/python_package_that_uses_tf2
        creating ./python_package_that_uses_tf2/python_package_that_uses_tf2/__init__.py
        creating folder ./python_package_that_uses_tf2/test
        creating ./python_package_that_uses_tf2/test/test_copyright.py
        creating ./python_package_that_uses_tf2/test/test_flake8.py
        creating ./python_package_that_uses_tf2/test/test_pep257.py

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

Package structure
-----------------

In this section, we will create or modify the following files.

.. code-block:: console
    :emphasize-lines: 5,6,10

    python_package_that_uses_tf2/
    |-- package.xml
    |-- python_package_that_uses_tf2
    |   |-- __init__.py
    |   |-- tf2_broadcaster_node.py
    |   `-- tf2_listener_node.py
    |-- resource
    |   `-- python_package_that_uses_tf2
    |-- setup.cfg
    |-- setup.py
    `-- test
        |-- test_copyright.py
        |-- test_flake8.py
        `-- test_pep257.py

Creating the broadcaster
------------------------

The broadcaster is made with the following piece of code.

:download:`tf2_broadcaster_node.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_broadcaster_node.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_broadcaster_node.py
   :language: python
   :lines: 24-
   :linenos:

Most of this should be familiar by now.

Focusing on the main novelty, we create the broadcaster with ``tf2_ros.TransformBroadcaster``. We need to input a
``rclpy.node.Node`` for the constructor, so we input it as ``self``.

As minor things, we have a tag for this frame as ``robot_name``. This could, for example, be a :program:`ros2`.
We leave it hard-coded for simplicity. We also keep time in the object's scope with ``timer_elapsed_time`` so that
it persists across callback calls.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_broadcaster_node.py
   :language: python
   :lines: 36-48
   :emphasize-lines: 5,9,12

We add two parameters to be able to adjust the trajectory properties if desired. Again, these could be configurable
parameters but we leave them hard-coded for simplicity.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_broadcaster_node.py
   :language: python
   :lines: 51-55
   :emphasize-lines: 4,5

We create the ``TransformStamped`` as follows. Our rotation is simple so we build Quaternion for the
rotation directly. We set the reference frame, ``header.frame_id``, as ``world``
and ``child_frame_id`` as this robot's frame.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_broadcaster_node.py
   :language: python
   :lines: 57-77

Finally, we send the transform with ``transform_broadcaster.sendTransform`` and update the local time counter for
the trajectory.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_broadcaster_node.py
   :language: python
   :lines: 79-83

Creating the listener
---------------------

The listener is made with the following piece of code.

:download:`tf2_listener_node.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_listener_node.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_listener_node.py
   :language: python
   :lines: 24-
   :linenos:

We use an instance of ``tf2_ros.TransformListener`` to manage the interaction with ``tf2`` for us. The only step that
might be unfamiliar is that you need to create an instance to a buffer which is used in the initializer of ``TransformListener``.

We also add the frame information that we want to listen to. This will be used in another step. One extension of this
would be to make this configurable as a parameter. We do not do that here to not increase complexity.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_listener_node.py
   :language: python
   :lines: 35-41
   :emphasize-lines: 2

We then create an instance of ``Timer`` and add the callback. In the callback, we use the buffer we created in the
previous step. We will call ``lookup_transform`` and it will need the parent frame tag, the child name tag,
and the time of lookup. We add exception handling in case the transform is not available or not available
in the time requested.

The object created with ``rclpy.time.Time()`` is `equivalent to a time of zero <https://github.com/ros2/rclpy/blob/938f4968bc742a77169e5d73d46619db34dbcc50/rclpy/rclpy/time.py#L43>`_, and ``lookup_transform`` `returns the latest transformation available <https://docs.ros2.org/foxy/api/tf2_ros/classtf2__ros_1_1Buffer.html#a3ab502cc1e8b608957a96ad350815aee>`_.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_listener_node.py
   :language: python
   :lines: 46-59
   :emphasize-lines: 4-7


The :file:`setup.py`
--------------------

We add the usual entry points.

:download:`setup.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/setup.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/setup.py
   :language: python
   :emphasize-lines: 27,28
   :linenos:

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: ../the_canonical_build_command.rst

Run Example
-----------

We can show the integrated example as follows.

.. tab-set::

    .. tab-item:: Terminal 1: tf2 broadcaster

        We start by running the broadcaster.

        .. code-block:: console

            ros2 run python_package_that_uses_tf2 tf2_broadcaster_node

    .. tab-item:: Terminal 2: tf2 listener

        Then, we run the listener.

        .. code-block:: console

            ros2 run python_package_that_uses_tf2 tf2_listener_node


Our broadcaster won't output anything to the screen.

The output of the listener will be as long as you allow it to be. The first line indicates a normal behavior, in which
subscribers to not have instant access to topics. That is properly handled by our listener. When available, you can see
the output.

.. code-block:: console

    [ERROR] [1762112353.248708886] [tf2_listener_node]: Could not transform world to robot_1: "world" passed to lookupTransform argument target_frame does not exist.
    [INFO] [1762112353.340386011] [tf2_listener_node]: Transform: geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1762112353, nanosec=335937928), frame_id='world'), child_frame_id='robot_1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.11448642511890406, y=-0.16399042186510032, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=-0.46236775104102995, w=0.8866882557005366)))
    [INFO] [1762112353.440609636] [tf2_listener_node]: Transform: geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1762112353, nanosec=436304803), frame_id='world'), child_frame_id='robot_1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.12455755609760884, y=-0.1564781621153285, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=-0.43428804928984416, w=0.9007740506053791)))

For now, you have to believe I'm right and that this means the broadcaster is sending a circular trajectory. We will
see more about this in visualization and simulation.