Publishers and Subscribers : using messages
===========================================

.. note::
   
   Except for the particulars of the :file:`setup.py` file, the way that publishers and subscribers in ROS2 work in Python, i.e. the explanation in this section, does not depend on :program:`ament_python` or :program:`ament_cmake`.

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

  cd ~/ros2_tutorial_workspace/src
  ros2 pkg create python_package_that_uses_the_messages \
  --build-type ament_python \
  --dependencies rclpy package_with_interfaces

Before we start exploring the elements of the package, let us

#. Create the publisher Node.
#. Create the subscriber Node.
#. Update the :file:`setup.py` so that :program:`ros2 run` finds these programs.

Create the publisher Node
-------------------------

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
   :emphasize-lines: 3, 11-14, 21-24, 26

When we built our :file:`package_with_interfaces` in the last section, what ROS2 did for us, among other things, was create a Python library called :file:`package_with_interfaces.msg` contaning the Python implementation of the :file:`AmazingQuote.msg`. Because of that, we can use it by importing it like so

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 24-26
   :emphasize-lines: 3

The publisher must be created with the :code:`Node.create_publisher(...)` method, and receives the three arguments we defined. 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 32-37
   :emphasize-lines: 3-6

The rule of thumb is that all of these three, including the :code:`qos_profile`, should be the same in the :code:`Publishers` and :code:`Subscribers` of the same topic.

+--------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------+
|:code:`msg_type`    |  A class, namely the message that will be used in the topic. In this case, :code:`AmazingQuote`.                                                         |
+--------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------+
|:code:`topic`       |  The topic through which the communication will occur. Can be arbitrarily chosen, but to make sense :code:`/amazing_quote`.                              |
+--------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------+
|:code:`qos_profile` |  The simplest interpretation for this parameter is the number of messages that will be stored if the :code:`spin(...)` takes to long to process them.    |
|                    |  (See more on  `docs for QoSProfile <https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html>`_.)                                 |
+--------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------+

Then, each message is handled much like any other class in Python. We instanteate and initialize the message as follows

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 42-44

Lastly, the message needs to be published using :code:`Node.publish(msg)`. 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 46

.. note::

   In general, the message will **NOT** be published instantenously after :code:`Node.publish()` is called. It is usually fast, but entirely dependent on :code:`rclpy.spin()` and how much work it is doing.

Create the subscriber Node
--------------------------

.. note::

         **TL:DR** Creating a subscriber

         #. Add new dependencies to :file:`package.xml`
         #. Import new messages :code:`from <package_name>.msg import <msg_name>`
         #. In a subclass of :code:`Node` 
         
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

- :code:`callback`, in which a method that receives a :code:`msg_type` and returns nothing is expected. For example, the :code:`amazing_quote_subscriber_callback`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py
   :language: python
   :lines: 40-57
   :emphasize-lines: 15-17

That callback method will be automatically called by ROS2, as one of the tasks performed by :code:`rclpy.spin(Node)`. Depending on the :code:`qos_profile`, it will not necessarily be the latest message.

.. note::

   The message will **ALWAYS** take some time between being published and being received by the subscriber. The speed in which that will happen will depend no only on this Node's :code:`rclpy.spin()`, but also on the :code:`rclpy.spin()` of the publisher node and the communication channel.

Update the :file:`setup.py`
---------------------------

As we already learned in :ref:`Making rosrun work`, we must adjust the :file:`setup.py` to refer to the Nodes we just created.

:download:`setup.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/setup.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/setup.py
   :language: python
   :linenos:
   :emphasize-lines: 21-25

Testing Publisher and Subscriber
--------------------------------

Whenever we need to open two or more terminal windows, remember that :ref:`Terminator is life`.

Let us open two terminals.

In the first terminal, we run

.. code:: console

  ros2 run python_package_that_uses_the_messages amazing_quote_publisher_node

Nothing in particular should happen now. The publisher is sending messages through the specific topic we defined, but we need at least one subscribed to interact with those messages.

Hence, in the second terminal, we run

.. code:: console

  ros2 run python_package_that_uses_the_messages amazing_quote_subscribed_node
 
which outputs

.. code:: console

  asd

.. note::

   If there are any issues with either the publisher or the subscriber, this connection will not work. In the next section, we'll see strategies to help us troubleshoot and understand the communication through topics.
