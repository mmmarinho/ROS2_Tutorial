Creating a Python Node from scratch (for :program:`ament_python`)
==============================================================

.. admonition:: **TL;DL** Making an :program:`ament_python` Node
         
         #. Modify :file:`package.xml` with any additional dependencies.
         #. Create the Node
         #. Modify the :file:`setup.py` file.

Let us add an additional Node to our :program:`ament_python` package that actually uses ROS2 functionality. 
These are the steps that must be taken, in general, to add a new node.

.. _Handling dependencies:

Handling dependencies (:file:`package.xml`)
-------------------------------------------

It is common for new Nodes to have additional dependencies, so we will cover that here. For any ROS2 package, we must modify the :file:`package.xml` to add the dependencies.

In this toy example, let us add the :code:`rclpy` as dependency, because it is the Python implementation of the :abbr:`RCL (ROS Client Library)`. All Nodes that use anything related to ROS2 will directly or indirectly depend on that library.

By no coincidence, the :file:`package.xml` has the :code:`.xml` extension, meaning that it is written in :abbr:`XML (Extensible Markup Language)`.

Let us add the dependency between the :code:`<license>` and :code:`<test_depend>` tags. This is not a strict requirement, but is where it commonly is for standard packages.

:download:`package.xml <../../ros2_tutorial_workspace/src/python_package_with_a_node/package.xml>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/package.xml
   :language: xml
   :linenos:
   :emphasize-lines: 10
  
After you modify the workspace, build once
------------------------------------------

.. warning:
 
   Depending on the particulars of the workspace you are (re)building, :program:`PyCharm` will only be able to recognize certain changes if it is restarted from a properly sourced terminal.

After you add a new dependency to :file:`package.xml`, nothing really changes in the workspace unless a new build is performed. 
When programming with new dependencies, unless you rebuild the workspace, :program:`PyCharm` will not recognize the libraries and autocomplete will not work.

So, run

.. include:: the_canonical_build_command.rst


Creating the Node
-----------------

In the directory :file:`src/python_package_with_a_node/python_package_with_a_node`, create a new file called :file:`print_forever_node.py`. Copy and paste the following contents into the file.

:download:`print_forever_node.py <../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/python_package_with_a_node/print_forever_node.py
   :language: python
   :linenos:
   :lines: 24-
   
By now, this should be enough for you to be able to run the node in :program:`PyCharm`. You can right click it and choose :guilabel:`D&ebug sample_python_node`. This will output

.. code :: console

    [INFO] [1683009340.877110693] [print_forever]: Printed 0 times.
    [INFO] [1683009341.336559942] [print_forever]: Printed 1 times.
    [INFO] [1683009341.836334639] [print_forever]: Printed 2 times.
    [INFO] [1683009342.336555088] [print_forever]: Printed 3 times.

To finish, press the :guilabel:`Stop` button or press :kbd:`CTRL+F2` on :program:`PyCharm`. The node will exit gracefully with

.. code :: console

   Process finished with exit code 0

.. _Making rosrun work:

Making :command:`ros2 run` work
-------------------------------

Even though you can run the new node in :program:`PyCharm`, we need an additional step to make it deployable in a place where :command:`ros2 run` can find it.

To do so, we modify the :code:`console_scripts` key in the :code:`entry_points` dictionary defined in :file:`setup.py`, to have our new node, as follows

.. hint:: 

   :code:`console_scripts` expects a :code:`list` of :code:`str` in a specific format. Hence, follow the format properly and don't forget the commas to separate elements in the :code:`list`.

:download:`setup.py <../../ros2_tutorial_workspace/src/python_package_with_a_node/setup.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_node/setup.py
   :language: python
   :linenos:
   :emphasize-lines: 24

The format is straightforward, as follows

==================================   ===================================================================================
:code:`print_forever_node`           The name of the node when calling it through :command:`ros2 run`.
:code:`python_package_with_a_node`   The name of the package.
:code:`print_forever_node`           The name of the script, without the :file:`.py` extension.
:code:`main`                         The function, within the script, that will be called. In general, :code:`main`.
==================================   ===================================================================================

Once again, we have to refresh the workspace so we run

.. include:: the_canonical_build_command.rst

And, with that, we can run

.. code :: console

  ros2 run python_package_with_a_node print_forever_node
   
which will output, as expected
 
.. code :: console
 
    [INFO] [1683010987.130432622] [print_forever]: Printed 0 times.
    [INFO] [1683010987.622780292] [print_forever]: Printed 1 times.
    [INFO] [1683010988.122731296] [print_forever]: Printed 2 times.
    [INFO] [1683010988.622735422] [print_forever]: Printed 3 times.

To stop, press :kbd:`CTRL+C` on the terminal and the Node will return gracefully.
