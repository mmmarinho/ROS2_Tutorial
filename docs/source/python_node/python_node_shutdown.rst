Shutting down cleanly
=====================

We press :code:`Ctrl-C` to stop a Node. The Node ending cleanly is essential for the correct operation of more complex software with many interconnected Nodes.

What :code:`Ctrl-C` actually does
---------------------------------

:code:`Ctrl-C` sends the :code:`SIGINT` signal to your Node. While :code:`rclpy.spin()` is running, :code:`rclpy` handles that signal itself, with two consequences:

1. :code:`spin()` raises an `ExternalShutdownException <https://docs.ros.org/en/jazzy/p/rclpy/generated/rclpy.executors.ExternalShutdownException.html>`_. If your :code:`except` clause does not catch it, the exception escapes :code:`main()` and Python dies.
2. The :code:`rclpy` **context is already shut down** by the signal handler. Calling :code:`rclpy.shutdown()` afterwards raises :code:`RCLError: rcl_shutdown already called on the given context`, crashing the Node.

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

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_with_shutdown_demos/python_package_with_shutdown_demos/bad_destroy_after_keyboard_interrupt_only.py
   :language: python

Endings that work, but are not recommended
------------------------------------------

The first survivor hedges the documented cleanup with a guard, which is what absorbs the double shutdown:

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_with_shutdown_demos/python_package_with_shutdown_demos/warn_ok_guard.py
   :language: python

The guard is a workaround for a crash you never need if you simply omit the cleanup.

The second pair uses :code:`rclpy.try_shutdown()`, which tolerates an already-dead context:

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_with_shutdown_demos/python_package_with_shutdown_demos/warn_try_shutdown.py
   :language: python

:code:`warn_try_shutdown_only` moves the :code:`rclpy.init()` inside the :code:`try` and drops the :code:`destroy_node()`.

Measured behaviour: 0 crashes in 15 runs each when stopped the way Nodes are normally stopped (:code:`ros2 run` + :code:`Ctrl-C`-equivalent group interrupt). However, when the :code:`SIGINT` is delivered directly to the Node's own process, the :code:`try_shutdown()` races :code:`rclpy`'s own handler and crashes about **1 in 3** times. It survives by winning a race, not by design.

The measured scoreboard
-----------------------

All numbers measured on ROS 2 Jazzy by starting each Node, waiting, and sending the :code:`SIGINT` a :code:`Ctrl-C` would send (to the process group), N runs per Node:

.. list-table::
   :header-rows: 1
   :widths: 30 35 15 20

   * - Executable
     - What you see at :code:`Ctrl-C`
     - Crashes
     - Verdict
   * - ``good_template``
     - clean exit
     - 0/3
     - recommended
   * - ``warn_ok_guard``
     - clean exit
     - 0/3
     - works, not recommended
   * - ``warn_keyboard_interrupt_only``
     - clean exit (the :code:`Exception` backstop saves it)
     - 0/3
     - works, not recommended
   * - ``warn_try_shutdown``
     - clean exit via :code:`ros2 run`; traceback when :code:`SIGINT` hits the Node process directly
     - 0/15 (1/3 with direct signal)
     - works, not recommended
   * - ``warn_try_shutdown_only``
     - same race as above
     - 0/15 (1/3 with direct signal)
     - works, not recommended
   * - ``bad_bare_shutdown``
     - RCLError traceback, exit code 1
     - 3/3
     - avoid
   * - ``bad_destroy_after_keyboard_interrupt_only``
     - uncaught ExternalShutdownException traceback, exit code 1
     - 3/3
     - avoid

Try them yourself
-----------------

.. code-block:: bash

   colcon build --packages-select python_package_with_shutdown_demos
   source install/setup.bash
   for n in good_template bad_bare_shutdown bad_destroy_after_keyboard_interrupt_only \
            warn_ok_guard warn_keyboard_interrupt_only warn_try_shutdown warn_try_shutdown_only; do
       setsid ros2 run python_package_with_shutdown_demos $n & pid=$!
       sleep 4
       kill -INT -$pid
       wait $pid
       echo "$n exit code $?"
   done

Each Node beats :code:`alive` on its logger once a second; only the :code:`Ctrl-C` behaviour differs. The :code:`kill -INT -$pid` (negative pid = the whole process group) reproduces what the terminal's :code:`Ctrl-C` does to :code:`ros2 run` and its child. Run the :code:`warn_try_shutdown` pair several times, or signal the Node process directly, to observe the race.
