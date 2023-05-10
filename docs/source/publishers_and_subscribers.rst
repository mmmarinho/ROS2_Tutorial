Publishers and Subscribers : using messages
===========================================

Finally, we reached the point where ROS2 becomes appealing. As you saw in the last section, we can easily create complex interface types using an easy and generic description.
We can use those to provide interprocess communication, i.e. two different programs talking to each other, which otherwise can be error-prone and very difficult to implement.

ROS2 works on a model in which any number of processes can communicate over a :code:`Topic` that only accepts one message type. Each topic is uniquely identified by a string.

Then

- A program that sends (publishes) information to the topic has a :code:`Publisher`.
- A program that reads (subscribes) information from a topic has a :code:`Subscriber`.

Each Node can have any number of :code:`Publishers` and :code:`Subscribers` and combination thereof, connecting to an arbitrary number of Nodes. This forms the so-called `ROS graph <https://docs.ros.org/en/humble/Concepts.html#quick-overview-of-ros-2-concepts>`_

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

.. code:: console

    ros2 topic echo -h

.. code:: console

  usage: ros2 topic echo [-h]
                         [--spin-time SPIN_TIME]
                         [-s] [--no-daemon]
                         [--qos-profile {unknown,system_default,sensor_data,services_default,parameters,parameter_events,action_status_default}]
                         [--qos-depth N]
                         [--qos-history {system_default,keep_last,keep_all,unknown}]
                         [--qos-reliability {system_default,reliable,best_effort,unknown}]
                         [--qos-durability {system_default,transient_local,volatile,unknown}]
                         [--csv] [--field FIELD]
                         [--full-length]
                         [--truncate-length TRUNCATE_LENGTH]
                         [--no-arr] [--no-str]
                         [--flow-style]
                         [--lost-messages]
                         [--no-lost-messages]
                         [--raw]
                         [--filter FILTER_EXPR]
                         [--once]
                         topic_name
                         [message_type]

  Output messages from a topic

  positional arguments:
    topic_name            Name of the ROS topic
                          to listen to (e.g.
                          '/chatter')
    message_type          Type of the ROS message
                          (e.g.
                          'std_msgs/msg/String')

  options:
    -h, --help            show this help message
                          and exit
    --spin-time SPIN_TIME
                          Spin time in seconds to
                          wait for discovery
                          (only applies when not
                          using an already
                          running daemon)
    -s, --use-sim-time    Enable ROS simulation
                          time
    --no-daemon           Do not spawn nor use an
                          already running daemon
    --qos-profile {unknown,system_default,sensor_data,services_default,parameters,parameter_events,action_status_default}
                          Quality of service
                          preset profile to
                          subscribe with
                          (default: sensor_data)
    --qos-depth N         Queue size setting to
                          subscribe with
                          (overrides depth value
                          of --qos-profile
                          option)
    --qos-history {system_default,keep_last,keep_all,unknown}
                          History of samples
                          setting to subscribe
                          with (overrides history
                          value of --qos-profile
                          option, default:
                          keep_last)
    --qos-reliability {system_default,reliable,best_effort,unknown}
                          Quality of service
                          reliability setting to
                          subscribe with
                          (overrides reliability
                          value of --qos-profile
                          option, default:
                          Automatically match
                          existing publishers )
    --qos-durability {system_default,transient_local,volatile,unknown}
                          Quality of service
                          durability setting to
                          subscribe with
                          (overrides durability
                          value of --qos-profile
                          option, default:
                          Automatically match
                          existing publishers )
    --csv                 Output all recursive
                          fields separated by
                          commas (e.g. for
                          plotting)
    --field FIELD         Echo a selected field
                          of a message. Use '.'
                          to select sub-fields.
                          For example, to echo
                          the position field of a
                          nav_msgs/msg/Odometry
                          message: 'ros2 topic
                          echo /odom --field
                          pose.pose.position'
    --full-length, -f     Output all elements for
                          arrays, bytes, and
                          string with a length >
                          '--truncate-length', by
                          default they are
                          truncated after '--
                          truncate-length'
                          elements with '...''
    --truncate-length TRUNCATE_LENGTH, -l TRUNCATE_LENGTH
                          The length to truncate
                          arrays, bytes, and
                          string to (default:
                          128)
    --no-arr              Don't print array
                          fields of messages
    --no-str              Don't print string
                          fields of messages
    --flow-style          Print collections in
                          the block style (not
                          available with csv
                          format)
    --lost-messages       DEPRECATED: Does
                          nothing
    --no-lost-messages    Don't report when a
                          message is lost
    --raw                 Echo the raw binary
                          representation
    --filter FILTER_EXPR  Python expression to
                          filter messages that
                          are printed. Expression
                          can use Python builtins
                          as well as m (the
                          message).
    --once                Print the first message
                          received and then exit.

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

.. code:: console

  ros2 run python_package_that_uses_the_messages amazing_quote_publisher_node


