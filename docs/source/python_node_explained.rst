The Python Node, explained
==========================

.. note::
   
   The way that a Python Node in :code:`ROS2` works, i.e. the explanation in this section, does not depend on the building with :program:`ament_python` or :program:`ament_cmake`.

In a strict sense, the :file:`print_forever_node.py` is not a minimal Node, but it does showcase most good practices in a Node that actually does something.

The imports
-----------

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 24-25
   
As in any :code:`Python` code, we have to import the libraries that we will use and specific modules/classes within those libraries. With :code:`rclpy`, there is no difference.

Making a subclass of :code:`Node`
---------------------------------

The current version of :code:`ROS2` behaves better when your custom node is a subclass of :code:`rclpy.node.Node`. That is achieved with 

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 28-33
   :emphasize-lines: 1,5
   
About inheritance in :code:`Python`, you can check the official documentation on `inheritance <https://docs.python.org/3/tutorial/classes.html#inheritance>`_ and on `super() <https://docs.python.org/3/library/functions.html#super>`_.

In more advanced nodes, inheritance does not cut it, but that is an advanced topic to be covered some other time.

Use a :code:`Timer` when using :code:`rclpy.spin()`
---------------------------------------------------

If the code relies on :code:`rclpy.spin()`, which is usually the easiest way to handle the :code:`ROS2` loop, periodic tasks must be handled by a `Timer <https://github.com/ros2/rclpy/blob/humble/rclpy/src/rclpy/timer.hpp>`_. 

To do so, have the node create it with the :code:`create_timer()` method, as follows.

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 31-35
   :emphasize-lines: 4

The method to be called is defined as follows

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 37-39
   :emphasize-lines: 1
   
In :code:`ROS2`, the logging methods, i.e. :code:`self.get_logger().info()`, depend on a Node. So, the capability to log using :code:`ROS2` Nodes is dependent on the scope in which that Node exists.
   
Don't forget :code:`rclpy.init()` and :code:`rclpy.spin()` 
----------------------------------------------------------

Nothing will happen unless these two methods are called. First, :code:`rclpy.init()` is going to initialize a bunch of :code:`ROS2` elements behind the curtains, whereas :code:`rclpy.spin()` will block the program. There are alternative ways to :code:`spin()`, but we will not discuss them right now.

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 42-55
   :emphasize-lines: 8,12
   
Have a :code:`try-catch` block for :code:`KeyboardInterrupt`
------------------------------------------------------------

In the current version of the `official ROS2 examples <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_ , for reasons beyond my comprehension this step is not followed.

However, when running Nodes either in the terminal or in :code:`PyCharm`, catching a :code:`KeyboardInterrupt` is the only reliable way to finish the Nodes cleanly. A :code:`KeyboardInterrupt` is emitted at a terminal by pressing :kbd:`CTRL+C`, whereas it is emitted by :code:`PyCharm` when pressing :guilabel:`Stop`.

That is particularly important when real robots need to be gracefully shutdown (otherwise they might unadvertedly start the evil robot uprising), but it also looks unprofessional when all your Nodes return with an ugly stack trace.

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 42-55
   :emphasize-lines: 7,13

Document your code with Docstrings
----------------------------------

As simple as a code might look for you right now, it needs to be documented for anyone you work with, including the future you. In a few weeks/months/years time, the :code:`BeStNoDeYouEvErWrote (TM)` might be indistinguishable from `Yautja Language <https://avp.fandom.com/wiki/Yautja_Language>`_.

Add as much description as possible to classes and methods, using the `Docstring Convention <https://peps.python.org/pep-0257/>`_.

Example for a class:

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 28-31
   :emphasize-lines: 2
   
Example for a method:

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 37-39
   :emphasize-lines: 2