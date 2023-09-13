The Python Node, explained
==========================

.. note::
   
   The way that a Python Node in ROS2 works, i.e. the explanation in this section, does not depend on the building with :program:`ament_python` or :program:`ament_cmake`.

In a strict sense, the :file:`print_forever_node.py` is not a minimal Node, but it does showcase most good practices in a Node that actually does something.

The imports
-----------

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py
   :language: python
   :lines: 24-25
   
As in any Python code, we have to import the libraries that we will use and specific modules/classes within those libraries. With :code:`rclpy`, there is no difference.

Making a subclass of :code:`Node`
---------------------------------

The current version of ROS2 behaves better when your custom Node is a subclass of :code:`rclpy.node.Node`. That is achieved with 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py
   :language: python
   :lines: 28-33
   :emphasize-lines: 1,5
   
About inheritance in Python, you can check the official documentation on `inheritance <https://docs.python.org/3/tutorial/classes.html#inheritance>`_ and on `super() <https://docs.python.org/3/library/functions.html#super>`_.

In more advanced nodes, inheritance does not cut it, but that is an advanced topic to be covered some other time.

.. _Use a Timer for periodic work:

Use a :code:`Timer` for periodic work (when using :code:`rclpy.spin()`)
-----------------------------------------------------------------------

.. admonition:: Tips for the future you

   If the code relies on :code:`rclpy.spin()`, a Timer must be used for periodic work.

In its most basic usage, periodic tasks in ROS2 must be handled by a `Timer <https://github.com/ros2/rclpy/blob/humble/rclpy/src/rclpy/timer.hpp>`_. 

To do so, have the node create it with the :code:`create_timer()` method, as follows.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py
   :language: python
   :lines: 31-35
   :emphasize-lines: 4

The method that is periodically called by the Timer is, in this case, as follows. We use :code:`self.get_logger().info()` to print to the terminal periodically.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py
   :language: python
   :lines: 37-39
   :emphasize-lines: 1
   
In ROS2, the logging methods, i.e. :code:`self.get_logger().info()`, are methods of the Node itself. So, the capability to log (print to the terminal) using ROS2 Nodes is dependent on the scope in which that Node exists.
   
Where the ROS2 magic happens: :code:`rclpy.init()` and :code:`rclpy.spin()` 
---------------------------------------------------------------------------

All the ROS2 magic happens in some sort of :code:`spin()` method. It is called this way because the :code:`spin()` method will constantly loop (or spin) through **items of work**, e.g. scheduled Timer callbacks. All the **items of work** will only be effectively executed when an **executor** runs through it. For simple Nodes, such as the one in this example, the **global** executor is implicitly used. You can read a bit more about that `here <https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html>`_.

Anyhow, the point is that nothing related to ROS2 will happen unless the two following methods are called. First, :code:`rclpy.init()` is going to initialize a bunch of ROS2 elements behind the curtains, whereas :code:`rclpy.spin()` will `block <https://en.wikipedia.org/wiki/Blocking_(computing)>`_ the program and, well, **spin** through Timer callbacks forever. There are alternative ways to :code:`spin()`, but we will not discuss them right now.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py
   :language: python
   :lines: 43-58
   :emphasize-lines: 8,12
   
Have a :code:`try-catch` block for :code:`KeyboardInterrupt`
------------------------------------------------------------

.. note:

   You can see more about this topic at :ref:`Python try catch`, in the preamble.

In the current version of the `official ROS2 examples <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_, for reasons beyond my comprehension, this step is not followed.

However, when running Nodes either in the terminal or in :program:`PyCharm`, catching a :code:`KeyboardInterrupt` is the only reliable way to finish the Nodes cleanly. A :code:`KeyboardInterrupt` is emitted at a terminal by pressing :kbd:`CTRL+C`, whereas it is emitted by :program:`PyCharm` when pressing :guilabel:`Stop`.

That is particularly important when real robots need to be gracefully shut down (otherwise they might inadvertently start the evil robot uprising), but it also looks unprofessional when all your Nodes return with an ugly stack trace.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py
   :language: python
   :lines: 43-58
   :emphasize-lines: 7,13

Document your code with Docstrings
----------------------------------

As simple as a code might look for you right now, it needs to be documented for anyone you work with, including the future you. In a few weeks/months/years time, the :code:`BeStNoDeYouEvErWrote (TM)` might be indistinguishable from `Yautja Language <https://avp.fandom.com/wiki/Yautja_Language>`_.

Add as much description as possible to classes and methods, using the `Docstring Convention <https://peps.python.org/pep-0257/>`_.

Example of a class:

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py
   :language: python
   :lines: 28-29
   :emphasize-lines: 2
   
Example of a method:

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py
   :language: python
   :lines: 37-38
   :emphasize-lines: 2
