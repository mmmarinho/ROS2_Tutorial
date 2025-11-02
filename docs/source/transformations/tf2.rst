Using :file:`tf2`
=================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. note::

    In :program:`ROS2`, the package for ``tf2`` is called ``tf2_ros``.

.. hint::

    An official ``tf2`` user guide is available here https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html.

Perhaps the most important benefit of ``tf2`` is how it connects with other :program:`ROS2` parts, such as ``rviz2``.

Create the package
------------------

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

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_broadcaster_node.py
   :language: python
   :lines: 24-
   :linenos:

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_tf2/python_package_that_uses_tf2/tf2_listener_node.py
   :language: python
   :lines: 24-
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