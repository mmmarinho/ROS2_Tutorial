Creating a Python Node from scratch (for :code:`ament_python`)
==============================================================

Let us add an additional Node to our :code:`ament_python` package that actually uses :code:`ROS2` functionality. 
These are the steps that must be taken, in general, to add a new node.

Handling dependencies
---------------------

It is common for new Nodes to have additional dependencies, so we will cover that here. For and :code:`ament_python` package, we must modify the :file:`package.xml` to add the dependencies.

In this toy example, let us add the :code:`rclpy` as dependency, because it is the :code:`Python` implementation of the :abbr:`RCL (ROS Client Library)`. All Nodes that use anything related to :code:`ROS2` will directly or indirectly depend on that library.

By no coincidence, the :file:`package.xml` has the :code:`.xml` extension, meaning that it is written in :abbr:`XML (Extensible Markup Language)`.

Let us add the dependency between the :code:`<license>` and :code:`<test_depend>` tags. This is not a strict requirement, but is where it commonly is for standard packages.

.. code:: xml

  <license>TODO: License declaration</license>

  <depend>rclpy</depend>

  <test_depend>ament_copyright</test_depend>

Creating the Node
-----------------

In the directory :file:`src/python_package_with_a_node/python_package_with_a_node`, create a new file called :file:`print_forever_node.py`.

Copy and paste the following contents into the file.

.. literalinclude:: ../scripts/print_forever_node.py
   :language: python
   :linenos:
   :lines:24-
   
.. note::
    
   At this stage, :code:`PyCharm` should be complaining that it couldn't find :code:`rclpy`. Can you guess why that is the case?


