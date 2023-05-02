Creating a Python Node from scratch (for :code:`ament_python`)
==============================================================

Let us add an additional Node to our :code:`ament_python` package that actually uses :code:`ROS2` functionality. 
These are the steps that must be taken, in general, to add a new node.

Handling dependencies
---------------------

It is common for new Nodes to have additional dependencies, so we will cover that here. For an :code:`ament_python` package, we must modify the :file:`package.xml` to add the dependencies.

In this toy example, let us add the :code:`rclpy` as dependency, because it is the :code:`Python` implementation of the :abbr:`RCL (ROS Client Library)`. All Nodes that use anything related to :code:`ROS2` will directly or indirectly depend on that library.

By no coincidence, the :file:`package.xml` has the :code:`.xml` extension, meaning that it is written in :abbr:`XML (Extensible Markup Language)`.

Let us add the dependency between the :code:`<license>` and :code:`<test_depend>` tags. This is not a strict requirement, but is where it commonly is for standard packages.

.. code-block:: xml
  :emphasize-lines: 3

  <license>TODO: License declaration</license>

  <depend>rclpy</depend>

  <test_depend>ament_copyright</test_depend>
  
After you modify the dependencies, build once
---------------------------------------------

After you add a new dependency to :file:`package.xml`, nothing really changes in the workspace unless a new build is performed. 
When programming with new dependencies, unless you rebuild the workspace, :code:`PyCharm` will not recognize the libraries and autocomplete will not work.

So, run

.. include:: the_canonical_build_command.rst

Creating the Node
-----------------

In the directory :file:`src/python_package_with_a_node/python_package_with_a_node`, create a new file called :file:`print_forever_node.py`.

Copy and paste the following contents into the file.

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 24-
   
By now, this should be enough for you to be able to run the node in :code:`PyCharm`. You can right click it and choose :guilabel:`D&ebug sample_python_node`. This will output

.. code:: bash

    [INFO] [1683009340.877110693] [print_forever]: Printed 0 times.
    [INFO] [1683009341.336559942] [print_forever]: Printed 0 times.
    [INFO] [1683009341.836334639] [print_forever]: Printed 0 times.
    [INFO] [1683009342.336555088] [print_forever]: Printed 0 times.

To finish, press the :guilabel:`Stop` button or press :kbd:`CTRL+F2`. The node will exit gracefully with

.. code:: bash

   Process finished with exit code 0
   
Making :command:`ros2 run` work
-------------------------------

Even though you can run the new node in :code:`PyCharm`, we need an additional step to make it deployable in a place where :command:`ros2 run` can find it.

To do so, we modify the :code:`entry_points` variable in :file:`setup.py`, from the original to

.. code-block:: python
     :emphasize-lines: 4
     
      entry_points={
          'console_scripts': [
              'sample_python_node = python_package_with_a_node.sample_python_node:main',
              'print_forever_node = python_package_with_a_node.print_forever_node:main'
          ],
      },

The format is straightforward. From left to right

- :code:`print_forever_node`: The name of the node when calling it through :command:`ros2 run`.
- :code:`python_package_with_a_node`: The name of the package.
- :code:`sample_python_node`: The name of the script, without the :file:`.py` extension.
- :code:`main`: The function, within the script, that will be called. In general, :code:`main`.

Once again, we have to refresh the workspace so we run

.. include:: the_canonical_build_command.rst

And, with that, we can run

.. code:: bash

  ros2 run python_package_with_a_node print_forever_node
   
which will output, as expected
 
.. code:: bash
 
    [INFO] [1683010987.130432622] [print_forever]: Printed 0 times.
    [INFO] [1683010987.622780292] [print_forever]: Printed 0 times.
    [INFO] [1683010988.122731296] [print_forever]: Printed 0 times.
    [INFO] [1683010988.622735422] [print_forever]: Printed 0 times.

To stop, press :kbd:`CTRL+C` and the Node will return gracefully.

The Node source code, explained
-------------------------------

In a strict sense, this new Node is not minimal, but it does showcase most good practices in a Node that actually does something.

1. The imports
~~~~~~~~~~~~~~

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 24-25
   
As in any :code:`Python` code, we have to import the libraries that we will use and specific modules/classes within those libraries. With :code:`rclpy`, there is no difference.

2. Making a subclass of :code:`Node`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The current version of :code:`ROS2` behaves better when your custom node is a subclass of :code:`rclpy.node.Node`. That is achieved with 

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 28-33
   :emphasize-lines: 1,5
   
About inheritance in :code:`Python`, you can check the official documentation on ` inheritance <https://docs.python.org/3/tutorial/classes.html#inheritance>`_ and on `super() <https://docs.python.org/3/library/functions.html#super>`_.

In more advanced nodes, inheritance doesn't cut it, but that is an advanced topic to be covered some other time.

3. Use a :code:`Timer` when using :code:`rclpy.spin()`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
   
In :code:`ROS2`, the logging methods depend on a Node. So, the capability to log using :code:`ROS2` Nodes is dependent on the scope in which that Node exists.
   
4. Don't forget :code:`rclpy.init()` and :code:`rclpy.spin()` 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Nothing will happen unless these two methods are called. First, :code:`rclpy.init()` is going to initialize a bunch of :code:`ROS2` elements behind the curtains, whereas :code:`rclpy.spin()` will block the program. There are alternative ways to :code:`spin()`, but we will not discuss them right now.

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 42-55
   :emphasize-lines: 8,12
   
5. *ALWAYS* have a :code:`try-catch` block for :code:`KeyboardInterrupt`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the current version of the `official ROS2 examples <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_, for reasons beyond my comprehension this step is not followed.

However, when running Nodes either in the terminal or in :code:`PyCharm`, catching a :code:`KeyboardInterrupt` is the only reliable way to finish the Nodes cleanly. A :code:`KeyboardInterrupt` is emitted at a terminal by pressing :kbd:`CTRL+C`, whereas it is emitted by :code:`PyCharm` when pressing :guilabel:`Stop`.

That is particularly important when real robots need to be gracefully shutdown (otherwise they might unadvertedly start the mechanical uprising), but it also looks unprofessional when all your Nodes return with an ugly stack trace.

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines: 42-55
   :emphasize-lines: 7,13

