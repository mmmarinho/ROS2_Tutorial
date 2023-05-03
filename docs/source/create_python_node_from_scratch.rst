Creating a Python Node from scratch (for :code:`ament_python`)
==============================================================

.. note::

  **TL;DL** When using :code:`ament_python`
  1. Modify :file:`package.xml` with any additional dependencies.
  2. Create the Node
  3. Modify the :file:`setup.py` file.
     

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
