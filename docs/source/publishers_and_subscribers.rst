Publishers and Subscribers : using messages
===========================================

.. note::
   
   Except for the particularities of the :file:`setup.py` file, the way that publishers and subscribers in :code:`ROS2` work, i.e. the explanation in this section, does not depend on :program:`ament_python` or :program:`ament_cmake`.

Finally, we reached the point where ROS2 becomes appealing. As you saw in the last section, we can easily create complex interface types using an easy and generic description.
We can use those to provide interprocess communication, i.e. two different programs talking to each other, which otherwise can be error-prone and very difficult to implement.

ROS2 works on a model in which any number of processes can communicate over a :code:`Topic` that only accepts one message type. Each topic is uniquely identified by a string.

Then

- A program that sends (publishes) information to the topic has a :code:`Publisher`.
- A program that reads (subscribes) information from a topic has a :code:`Subscriber`.

Each Node can have any number of :code:`Publishers` and :code:`Subscribers` and combination thereof, connecting to an arbitrary number of Nodes. This forms part of the connections in the so-called `ROS graph <https://docs.ros.org/en/humble/Concepts.html#quick-overview-of-ros-2-concepts>`_.

Create the package and its elements
-----------------------------------

First, let us create an :program:`ament_python` package that depends on our newly developed :file:`packages_with_interfaces` and build from there.

.. code:: console

  ros2 pkg create python_package_that_uses_the_messages \
  --build-type ament_python \
  --dependencies rclpy package_with_interfaces

Before we start exploring the elements of the package, let us

#. Create the publisher Node.
#. Create the subscriber Node.
#. Update the :file:`setup.py` so that :program:`ros2 run` finds these programs.

Create the publisher Node
^^^^^^^^^^^^^^^^^^^^^^^^^
.. note::

         **TL:DR** Creating a publisher

         #. Add new dependencies to :file:`package.xml`
         #. Import new messages :code:`from <package_name>.msg import <msg_name>`
         #. In a subclass of :code:`Node`
         
            #. create a publisher with :code:`self.publisher = self.create_publisher(...)`
            #. Send messages with :code:`self.publisher.publish(....)`
            
         #. Add the new Node to :file:`setup.py`

For the publisher, create a file in :file:`python_package_that_uses_the_messages/python_package_that_uses_the_messages` called :file:`amazing_quote_publisher_node.py`, with the following contents

:download:`amazing_quote_publisher_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 3, 11, 18-21, 23

When we built our :file:`package_with_interfaces` in the last section, what ROS2 did for us, among other things, was create a Python library called :file:`package_with_interfaces.msg` contaning the Python implementation of the :file:`AmazingQuote.msg`. Because of that, we can use it by importing it like so

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 24-26
   :emphasize-lines: 3

The publisher must be created with the :code:`Node.create_publisher(...)` method, and receives the three arguments we defined. 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 32-37
   :emphasize-lines: 3

The rule of thumb is that all of these three, including the :code:`qos_profile`, should be the same in the :code:`Publishers` and :code:`Subscribers` of the same topic.

- :code:`msg_type: {__class__}` A class, namely the message that will be used in the topic. In this case, :code:`AmazingQuote`.
- :code:`topic: str`: The topic through which the communication will occur. Can be arbitrarily chosen, but to make sense :code:`/amazing_quote`.
- :code:`qos_profile: QoSProfile | int`: The simplest interpretation for this parameter is the number of messages that will be saved if the :code:`spin(...)` takes to long to process them. The long story is available at the `docs for QoSProfile <https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html>`_.

Then, each message is handled much like any other class in Python. We instanteate and initialize the message as follows

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 41-44

Lastly, the message needs to be published using :code:`Node.publish(msg)`. 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 46

.. note::

   In general, the message will **NOT** be published instantenously after :code:`Node.publish()` is called. It is usually fast, but entirely dependent on :code:`rclpy.spin()` and how much work it is doing.

Create the subscriber Node
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

         **TL:DR** Creating a subscriber

         #. Add new dependencies to :file:`package.xml`
         #. Import new messages :code:`from <package_name>.msg import <msg_name>`
         #. In a subclass of :code:`Node`, 
         
                #. create a callback :code:`def callback(self, msg):`
                #. create a subscriber :code:`self.subscriber = self.create_subscription(...)`
                
         #. Add the new Node to :file:`setup.py`

For the subscriber Node, create a file in :file:`python_package_that_uses_the_messages/python_package_that_uses_the_messages` called :file:`amazing_quote_subscriber_node.py`, with the following contents

:download:`amazing_quote_subscriber_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 3, 11-15, 17-34
   
Similarly to the publisher, in the subscriber we start by importing the message in question

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py
   :language: python
   :lines: 24-26
   :emphasize-lines: 3

Then, in our subclass of :code:`Node`, we call :code:`Node.create_publisher(...)` as follows

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py
   :language: python
   :lines: 32-38
   :emphasize-lines: 3-7
   
where the only difference with respect to the publisher is the third argument, namely

- :code:`callback`



Update the :file:`setup.py`
^^^^^^^^^^^^^^^^^^^^^^^^^^

:download:`setup.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/setup.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/setup.py
   :language: python
   :linenos:
   :emphasize-lines: 21-25

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


