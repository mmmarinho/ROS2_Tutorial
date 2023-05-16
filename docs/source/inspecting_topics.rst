Inspecting topics (:program:`ros2 topic`)
=========================================

ROS2 has a tool to help us inspect topics. This is used with comsiderable frequency in practice to troubleshoot and speed up the development of publishers and subscribers. As usual, we can more information on this tool as follows.

.. code:: console

    ros2 topic -h

which outputs the detailed information of the tool, as shown below. In particular, the highlighed fields are used quite frequently in practice.

.. code-block:: console
    :emphasize-lines: 19,21-24

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

Start a publisher
-----------------

During the development of a publisher, it is extremely useful to be able to check if topics are being properly made before we venture into making the subscribers. To see some of the tools for this job, we start by running the publisher Node we wrote in the last section.

.. warning::

   Be sure to terminate the Nodes we used in the past section before proceeding (e.g. with :kbd:`CTLR+C`), otherwise the output will look different from what is described here.

.. code:: console

  ros2 run python_package_that_uses_the_messages amazing_quote_publisher_node 

Getting all topics with :program:`ros2 topic list`
--------------------------------------------------

In particular when there are many topics, it is difficult to remember every name. To see all currently active topics, we can run

.. code:: console

   ros2 topic list
   
which, in this case, outputs

.. code-block:: console
    :emphasize-lines: 1

    /amazing_quote
    /parameter_events
    /rosout

showing, in particular, the :code:`/amazing_quote` topic what we were looking for.

:program:`grep` is your new best friend
---------------------------------------

.. note::

   If you want more information on :program:`grep`, check the `Ubuntu Manpage <https://manpages.ubuntu.com/manpages/bionic/en/man1/grep.1.html>`_

When the list of topics is too large, we can use :program:`grep` to help filter the output. E.g.

.. code:: console

   ros2 topic list | grep quote
   
which outputs only the lines that contain :code:`quote`, that is

.. code:: console

   /amazing_quote
   
Getting quick info with :program:`ros2 topic info`
--------------------------------------------------

To get some quick information on a topic we can run

.. code:: console

    ros2 topic info /amazing_quote

which outputs the message type and the number of publishers and subscribers connected to that topic

.. code-block:: console
    :emphasize-lines: 2

    Type: package_with_interfaces/msg/AmazingQuote
    Publisher count: 1
    Subscription count: 0

Checking topic contents with :program:`ros2 topic echo`
-------------------------------------------------------

The :program:`ros2 topic echo` is the main tool that we can use to inspect topic activity. We can check all the options of :program:`ros2 topic echo` with the command below. The output is quite long so it's not replicated here.

.. code:: console

    ros2 topic echo -h

To inspect the topic whose name we alredy know, we run

.. code:: console

    ros2 topic echo /amazing_quote 

which outputs the following

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

:program:`grep` is still your best friend
-----------------------------------------

Whenever the topic is too crowded or the messages too fast, it might be difficult to pinpoint a single field we are looking for. In that case, :program:`grep` can also help. 

For example let us say that we want to see only the :code:`id` fields of the messages. We can do

.. code:: console

    ros2 topic echo /amazing_quote | grep id

which will output only the lines with that pattern, e.g.

.. code:: console

    id: 1550
    id: 1551
    id: 1552
    id: 1553

Measuring publishing frequency with :program:`ros2 topic hz`
------------------------------------------------------------

There are situations in which we are interested in knowing if the topics are receiving messages at an expected rate, without particular interest in the contents of the messages. We can do so with

.. code:: console

   ros2 topic hz /amazing_quote
   
which will output, after some time,

.. code:: console

       WARNING: topic [/amazing_quote] does not appear to be published yet
    average rate: 2.000
        min: 0.500s max: 0.500s std dev: 0.00007s window: 4
    average rate: 2.000
        min: 0.500s max: 0.500s std dev: 0.00013s window: 7
    average rate: 2.000
        min: 0.500s max: 0.500s std dev: 0.00011s window: 9

We must wait for a while until messages are received so that the tool can measure the frequency properly. You probably have noticed that the frequency measured by :program:`ros2 topic hz` is compatible with the period of the :code:`Timer` in our publisher Node.

Stop the publisher
------------------

Now we have exhausted all relevant tools that can give us information related to the publisher. Let us close the publisher with :kbd:`CTLR+C` so that we can evaluate how these tools can help us analise a subscriber.

Start the subscriber and get basic info
---------------------------------------

.. code:: console

  ros2 run python_package_that_uses_the_messages amazing_quote_subscriber_node 

When only the subscriber is running, we can still get the basic info on the topic, e.g. 

.. code:: console

   ros2 topic list
   
which also outputs

.. code-block:: console
    :emphasize-lines: 1

    /amazing_quote
    /parameter_events
    /rosout

and 

.. code:: console

    ros2 topic info /amazing_quote

which, differently from before, outputs

.. code-block:: console
    :emphasize-lines: 3

    Type: package_with_interfaces/msg/AmazingQuote
    Publisher count: 0
    Subscription count: 1

Testing your subscribers with :program:`ros2 topic pub`
-------------------------------------------------------

.. note::

   To improve readability, the command is using the escape character ``\``. You can see more on this at the `bash docs <https://www.gnu.org/software/bash/manual/bash.html#Escape-Character>`_.

.. code-block:: console
   :emphasize-lines: 4-6

   ros2 topic pub /amazing_quote \
   package_with_interfaces/msg/AmazingQuote \
   "{ \
   id: 2008, \
   quote: uhhh. I feel funny., \
   philosopher_name: David After Dentist \
   }"

