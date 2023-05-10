Publishers and Subscribers : using messages
===========================================

Finally, we reached the point where ROS2 becomes appealing. As you saw in the last section, we can easily create complex interface types using an easy and generic description.
We can use those to provide interprocess communication, i.e. two different programs talking to each other, which otherwise can be error-prone and very difficult to implement.

ROS2 works on a model in which any number of processes can communicate over a :code:`Topic` that only accepts one message type. Each topic is uniquely identified by a string.

Then

- A program that sends (publishes) information to the topic has a :code:`Publisher`.
- A program that reads (subscribes) information from a topic has a :code:`Subscriber`.

Each Node can have any number of :code:`Publishers` and :code:`Subscribers` and combination thereof, connecting to an arbitrary number of Nodes. This forms part of the connections in the so-called `ROS graph <https://docs.ros.org/en/humble/Concepts.html#quick-overview-of-ros-2-concepts>`_.

Create the package
------------------

First, let us create an :program:`ament_python` package that depends on our newly developed :file:`packages_with_interfaces` and build from there.

.. code:: console

  ros2 pkg create python_package_that_uses_the_messages \
  --build-type ament_python \
  --dependencies rclpy package_with_interfaces
  
Create the publisher Node
--------------------------

:download:`amazing_quote_publisher_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 3, 11, 18-21, 23

Create the subscriber Node
--------------------------

:download:`amazing_quote_subscriber_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 3, 11-15, 17-34

Inspecting topics with :program:`ros2 topic`
--------------------------------------------

.. code:: console

    ros2 topic -h

.. code:: console

    usage: ros2 topic [-h]
                      [--include-hidden-topics]
                      Call `ros2 topic <command>
                      -h` for more detailed usage.
                      ...

    Various topic related sub-commands

    options:
      -h, --help            show this help message
                            and exit
      --include-hidden-topics
                            Consider hidden topics
                            as well

    Commands:
      bw     Display bandwidth used by topic
      delay  Display delay of topic from timestamp in header
      echo   Output messages from a topic
      find   Output a list of available topics of a given type
      hz     Print the average publishing rate to screen
      info   Print information about a topic
      list   Output a list of available topics
      pub    Publish a message to a topic
      type   Print a topic's type

      Call `ros2 topic <command> -h` for more detailed usage.

You can check all the options of :program:`ros2 topic echo` with the command below. The output is quite long so it's not replicated here.

.. code:: console

    ros2 topic echo -h

Checking our publisher with :program:`ros2 topic`
-------------------------------------------------

.. code:: console

  ros2 run python_package_that_uses_the_messages amazing_quote_publisher_node 

.. code:: console

    ros2 topic echo /amazing_quote 

.. code:: console

  id: 6
  quote: Use the force, Pikachu!
  philosopher_name: Uncle Ben
  ---
  id: 7
  quote: Use the force, Pikachu!
  philosopher_name: Uncle Ben
  ---
  id: 8
  quote: Use the force, Pikachu!
  philosopher_name: Uncle Ben
  ---
  id: 9
  quote: Use the force, Pikachu!
  philosopher_name: Uncle Ben
  ---
  id: 10
  quote: Use the force, Pikachu!
  philosopher_name: Uncle Ben
  ---
  id: 11
  quote: Use the force, Pikachu!
  philosopher_name: Uncle Ben
  ---

Testing Publisher and Subscriber simultaneously
-----------------------------------------------

.. code:: console

  ros2 run python_package_that_uses_the_messages amazing_quote_publisher_node


