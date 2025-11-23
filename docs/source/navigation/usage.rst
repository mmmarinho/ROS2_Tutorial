Using ``nav2``
==============

.. versionadded:: Jazzy

   This section.

.. include:: ../the_topic_is_under_heavy_construction.rst

There is a plethora of online examples of ``nav2``, with varying levels of detail and quality. My approach will top-down,
where we will see how illustrative examples and then walk through their contents.

Navigation on a known map
+++++++++++++++++++++++++

.. seealso::

    Official documentation: https://docs.nav2.org/tutorials/docs/navigation2_on_real_turtlebot3.html

In this example, our interest is looking at how navigation can be done with built-in tools, using a known *map*. A
map will have many definitions and the literature about it is extensive. For now, we can think of the map as something
usual. It points out where things are, where passable (such as roads) regions are, and possibly hazards and other
relevant objects.

In this example, a `TurtleBot3 <https://www.turtlebot.com/turtlebot3/>`_ will be used. As part of ``nav2_bringup``, there
is a rather complete example that we can utilize, namely :file:`tb3_simulation_launch.py`.

.. warning::

    The ``sigterm_timeout`` flag is particularly important. Given that some devices we use might not be powerful enough
    for all processes to finish cleanly within 5 or 10 seconds, we should add this to prevent a ``SIGTERM`` from
    being sent to the nodes. If nodes are not terminated correctly they can leave connections open, linger indefinitely,
    and cause extremely difficult-to-debug situations. This can also be dangerous when real robots are used.

.. code-block:: console

    ros2 launch nav2_bringup tb3_simulation_launch.py use_sim_time:=True headless:=False sigterm_timeout:=120

.. warning::

    The ``jazzy`` version of this example shows a number of errors when shutting down. This seems to be causing issues
    when the example is run repeatedly. 


Navigation with SLAM
++++++++++++++++++++

.. seealso::

    Official documentation: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html#navigation2-with-slam

.. code-block:: console

    ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True sigterm_timeout:=120