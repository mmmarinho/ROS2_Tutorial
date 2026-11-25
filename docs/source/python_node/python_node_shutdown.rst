Shutting down cleanly
=====================

We press :code:`Ctrl-C` to stop a Node. The Node ending cleanly is essential for the correct operation of more complex software with many interconnected Nodes.

What :code:`Ctrl-C` actually does
---------------------------------

:code:`Ctrl-C` sends the :code:`SIGINT` signal to your Node. While :code:`rclpy.spin()` is running, :code:`rclpy` handles that signal itself, with three consequences:

1. :code:`spin()` raises an `ExternalShutdownException <https://docs.ros.org/en/jazzy/p/rclpy/generated/rclpy.executors.ExternalShutdownException.html>`_. If your :code:`except` clause does not catch it, the exception escapes :code:`main()` and Python dies.
2. The :code:`rclpy` **context is already shut down** by the signal handler. Calling :code:`rclpy.shutdown()` afterwards raises :code:`RCLError: rcl_shutdown already called on the given context`, crashing the Node.
3. :code:`destroy_node()` serves no purpose in the shutdown process, so I will omit them in the examples. It does not crash the node, but it is not needed either.

The Node used by every ending
-----------------------------

All the endings below run the same tiny Node, kept in a module that every example imports:

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_with_shutdown_demos/python_package_with_shutdown_demos/_heartbeat.py
   :language: python

The recommended ending
----------------------

Catch both exit paths and do not shut the context down.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_with_shutdown_demos/python_package_with_shutdown_demos/good_template.py
   :language: python

This is the only ending you need. Everything below exists because the ROS 2 documentation and Q&A threads show other endings.

Endings that crash
------------------

The ending recommended by `the ROS 2 documentation <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_ cleans up in a :code:`finally` block:

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_with_shutdown_demos/python_package_with_shutdown_demos/bad_bare_shutdown.py
   :language: python

Under :code:`Ctrl-C` the :code:`finally` block executes *after* :code:`rclpy`'s signal handler has already shut the context down, so :code:`rclpy.shutdown()` explodes:

.. code-block:: text

   rclpy.exceptions.RCLError: rcl_shutdown already called on the given context

A second crashing ending catches only :code:`KeyboardInterrupt`, so the :code:`ExternalShutdownException` raised by :code:`spin()` has nowhere to go:

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_with_shutdown_demos/python_package_with_shutdown_demos/bad_uncaught_external_shutdown.py
   :language: python

Endings that work, but are not recommended
------------------------------------------

The first survivor hedges the documented cleanup with a guard, which is what absorbs the double shutdown:

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_with_shutdown_demos/python_package_with_shutdown_demos/warn_ok_guard.py
   :language: python

The guard is a workaround for a crash you never need if you simply omit the :code:`shutdown()`.

Note that :code:`rclpy.try_shutdown()` tolerates an already-dead context.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_with_shutdown_demos/python_package_with_shutdown_demos/warn_try_shutdown.py
   :language: python





