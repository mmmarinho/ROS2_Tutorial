Publishers and Subscribers: using messages
===========================================

Finally, we reached the point where :program:`ROS2` becomes appealing. As you saw in the last section, we can easily create complex interface types using an easy and generic description.
We can use those to provide `interprocess communication <https://en.wikipedia.org/wiki/Inter-process_communication>`_, i.e. two different programs talking to each other, which otherwise can be error-prone and very difficult to implement.

:program:`ROS2` messages work on a model in which any number of processes can communicate over a :code:`Topic` that only accepts one message type. Each topic is uniquely identified by a string.

Then

- A program that sends (publishes) information to the topic has one or more :code:`Publisher` \(s).
- A program that reads (subscribes) information from a topic has one or more :code:`Subscriber` \(s).

Each Node can have any number of :code:`Publishers` and :code:`Subscribers` and a combination thereof, connecting to an arbitrary number of Nodes. This forms part of the connections in the so-called `ROS graph <https://docs.ros.org/en/humble/Concepts.html#quick-overview-of-ros-2-concepts>`_. An example is shown below.

Diagram
-------

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    graph LR;
    A[Publisher #1] --> B[Topic]
    C[Publisher #2] --> B
    B --> D[Subscriber #1]

.. note::

   This is an abstraction. As long as the information flows in this manner, it does not mean that an entity called ``topic`` must exist.
   In :program:`ROS2`, this type of communication happens, in fact, peer-to-peer.

Package structure
-----------------

This will be the structure of the package. The main elements are highlighted.

.. code-block:: console
    :emphasize-lines: 5,6

        python_package_that_uses_the_messages/
        |-- package.xml
        |-- python_package_that_uses_the_messages
        |   |-- __init__.py
        |   |-- amazing_quote_publisher_node.py
        |   `-- amazing_quote_subscriber_node.py
        |-- resource
        |   `-- python_package_that_uses_the_messages
        |-- setup.cfg
        |-- setup.py
        `-- test
            |-- test_copyright.py
            |-- test_flake8.py
            `-- test_pep257.py


Create the package
------------------

First, let us create an :program:`ament_python` package that depends on our newly developed :file:`packages_with_interfaces` and build from there.

.. code:: console

  cd ~/ros2_tutorial_workspace/src
  ros2 pkg create python_package_that_uses_the_messages \
  --build-type ament_python \
  --dependencies rclpy package_with_interfaces

.. dropdown:: ros2 pkg create output

   .. code :: console

        going to create a new package
        package name: python_package_that_uses_the_messages
        destination directory: /root/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['rclpy', 'package_with_interfaces']
        creating folder ./python_package_that_uses_the_messages
        creating ./python_package_that_uses_the_messages/package.xml
        creating source folder
        creating folder ./python_package_that_uses_the_messages/python_package_that_uses_the_messages
        creating ./python_package_that_uses_the_messages/setup.py
        creating ./python_package_that_uses_the_messages/setup.cfg
        creating folder ./python_package_that_uses_the_messages/resource
        creating ./python_package_that_uses_the_messages/resource/python_package_that_uses_the_messages
        creating ./python_package_that_uses_the_messages/python_package_that_uses_the_messages/__init__.py
        creating folder ./python_package_that_uses_the_messages/test
        creating ./python_package_that_uses_the_messages/test/test_copyright.py
        creating ./python_package_that_uses_the_messages/test/test_flake8.py
        creating ./python_package_that_uses_the_messages/test/test_pep257.py

        [WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
        It is recommended to use one of the ament license identifiers:
        Apache-2.0
        BSL-1.0
        BSD-2.0
        BSD-2-Clause
        BSD-3-Clause
        GPL-3.0-only
        LGPL-3.0-only
        MIT
        MIT-0

Overview
--------

.. note::

   By no coincidence, I am using the terminology Node *with* a publisher, and Node *with* a subscriber. In general, each Node will have a combination of publishers, subscribers, and other interfaces.

Before we start exploring the elements of the package, let us

#. Create the Node with a publisher.
#. Create the Node with a subscriber.
#. Update the :file:`setup.py` so that :program:`ros2 run` finds these programs.

.. _Create a publisher:

Create the Node with a publisher
--------------------------------

.. admonition:: **TL;DR** Creating a publisher

               #. Add new dependencies to :file:`package.xml`
               #. Import new messages :code:`from <package_name>.msg import <msg_name>`
               #. In a subclass of :code:`Node`
               
                  #. Create a publisher with :code:`self.publisher = self.create_publisher(...)`
                  #. Send messages with :code:`self.publisher.publish(....)`
                  
               #. Add the new Node to :file:`setup.py`

For the publisher, create a file called :file:`amazing_quote_publisher_node.py`, with the following contents

:download:`~/ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 3, 12-15, 25-28, 30

When we built our :file:`package_with_interfaces` in the last section, what ROS2 did for us, among other things, was create a Python library called :file:`package_with_interfaces.msg` containing the Python implementation of the :file:`AmazingQuote.msg`. Because of that, we can use it by importing it like so

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 24-26
   :emphasize-lines: 3

The publisher must be created with the :code:`Node.create_publisher(...)` method, which has the three parameters defined in the :ref:`publisher and subscriber parameter table`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 35-38

.. _publisher and subscriber parameter table:

+--------------------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
|:code:`msg_type`    |  A class, namely the message that will be used in the topic. In this case, :code:`AmazingQuote`.                                                                                                |
+--------------------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
|:code:`topic`       |  The topic through which the communication will occur. Can be arbitrarily chosen, but to make sense :code:`/amazing_quote`.                                                                     |
+--------------------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
|:code:`qos_profile` |  The simplest interpretation for this parameter is the maximum number of messages that will be stored in a buffer if your node (including :code:`spin(...)`) takes too long to process them.    |
|                    |  (See more on  `docs for QoSProfile <https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html>`_.)                                                                        |
+--------------------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+

.. warning::

   All the arguments in :ref:`publisher and subscriber parameter table` should be *EXACTLY* the same in the :code:`Publishers` and :code:`Subscribers` of the same topic.

Then, each message is handled much like any other class in Python. We instantiate and initialize the message as follows

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 48-51

Lastly, the message needs to be published using :code:`Node.publish(msg)`. 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_publisher_node.py
   :language: python
   :lines: 53

.. note::

   In general, the message will **NOT** be published instantaneously after :code:`Node.publish()` is called. It is usually fast, but entirely dependent on :code:`rclpy.spin()` and how much work it is doing.

Create the Node with a subscriber
---------------------------------

.. admonition:: **TL;DR** Creating a subscriber

               #. Add new dependencies to :file:`package.xml`
               #. Import new messages :code:`from <package_name>.msg import <msg_name>`
               #. In a subclass of :code:`Node`
               
                  #. Create a callback :code:`def callback(self, msg):`
                  #. Create a subscriber :code:`self.subscriber = self.create_subscription(...)`
                  
               #. Add the new Node to :file:`setup.py`

For the subscriber Node, create a file in :file:`python_package_that_uses_the_messages/python_package_that_uses_the_messages` called :file:`amazing_quote_subscriber_node.py`, with the following contents

:download:`~/ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 3, 11-15, 17-31
   
Similarly to the publisher, in the subscriber, we start by importing the message in question

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py
   :language: python
   :lines: 24-26
   :emphasize-lines: 3

Then, in our subclass of :code:`Node`, we call :code:`Node.create_subscription(...)` as follows

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py
   :language: python
   :lines: 34-38
   :emphasize-lines: 4
   
where the only difference with respect to the publisher is the third argument, namely :code:`callback`, in which a method that receives a :code:`msg_type` and returns nothing is expected. For example, the :code:`amazing_quote_subscriber_callback`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/python_package_that_uses_the_messages/amazing_quote_subscriber_node.py
   :language: python
   :lines: 40-53
   :emphasize-lines: 8,12,14

That callback method will be automatically called by ROS2, as one of the tasks performed by :code:`rclpy.spin(Node)`. Depending on the :code:`qos_profile`, it will not necessarily be the latest message.

.. note::

   The message will **ALWAYS** take some time between being published and being received by the subscriber. The speed in which that will happen will depend not only on this Node's :code:`rclpy.spin()`, but also on the :code:`rclpy.spin()` of the publisher node and the communication channel.

Update the :file:`setup.py`
---------------------------

As we already learned in :ref:`Making rosrun work`, we must adjust the :file:`setup.py` to refer to the Nodes we just created.

:download:`~/ros2_tutorial_workspace/src/python_package_that_uses_the_messages/setup.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/setup.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_messages/setup.py
   :language: python
   :linenos:
   :emphasize-lines: 21-25

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: the_canonical_build_command.rst

Testing Publisher and Subscriber
--------------------------------

Whenever we need to open two or more terminal windows, remember that :ref:`Terminator is life`.

Let us open two terminals.

In the first terminal, we run

.. code:: console

  ros2 run python_package_that_uses_the_messages amazing_quote_publisher_node

Nothing, in particular, should happen now. The publisher is sending messages through the specific topic we defined, but we need at least one subscriber to interact with those messages.

Hence, in the second terminal, we run

.. code:: console

  ros2 run python_package_that_uses_the_messages amazing_quote_subscriber_node
 
which outputs

.. code:: console

    [INFO] [1753664072.638312553] [amazing_quote_subscriber_node]:
            I have received the most amazing of quotes.
            It says

                   'Use the force, Pikachu!'

            And was thought by the following genius

                -- Uncle Ben

            This latest quote had the id=37.

    [INFO] [1753664073.121886428] [amazing_quote_subscriber_node]:
            I have received the most amazing of quotes.
            It says

                   'Use the force, Pikachu!'

            And was thought by the following genius

                -- Uncle Ben

            This latest quote had the id=38.



.. note::

   If there are any issues with either the publisher or the subscriber, this connection will not work. In the next section, we'll see strategies to help us troubleshoot and understand communication through topics.
   
.. warning::

   Unless instructed otherwise, the publisher does **NOT** wait for a subscriber to connect before it starts publishing the messages. As shown in the case above, the first message we received started with `id>0`. If we delayed longer to start the publisher, we would have received later messages only.
   
Let's close each node with :kbd:`CTRL+C` on each terminal before we proceed to the next tutorial.
