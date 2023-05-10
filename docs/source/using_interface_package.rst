Using interfaces from another package
=====================================

.. code:: console

  ros2 pkg create python_package_that_uses_the_interfaces \
  --build-type ament_python \
  --dependencies rclpy package_with_interfaces
  

.. code:: console

  going to create a new package
  package name: python_package_that_uses_the_interfaces
  destination directory: /home/murilo/git/ROS2_Tutorial/ros2_tutorial_workspace/src
  package format: 3
  version: 0.0.0
  description: TODO: Package description
  maintainer: ['murilo <murilomarinho@ieee.org>']
  licenses: ['TODO: License declaration']
  build type: ament_python
  dependencies: ['rclpy', 'package_with_interfaces']
  creating folder ./python_package_that_uses_the_interfaces
  creating ./python_package_that_uses_the_interfaces/package.xml
  creating source folder
  creating folder ./python_package_that_uses_the_interfaces/python_package_that_uses_the_interfaces
  creating ./python_package_that_uses_the_interfaces/setup.py
  creating ./python_package_that_uses_the_interfaces/setup.cfg
  creating folder ./python_package_that_uses_the_interfaces/resource
  creating ./python_package_that_uses_the_interfaces/resource/python_package_that_uses_the_interfaces
  creating ./python_package_that_uses_the_interfaces/python_package_that_uses_the_interfaces/__init__.py
  creating folder ./python_package_that_uses_the_interfaces/test
  creating ./python_package_that_uses_the_interfaces/test/test_copyright.py
  creating ./python_package_that_uses_the_interfaces/test/test_flake8.py
  creating ./python_package_that_uses_the_interfaces/test/test_pep257.py

  [WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
  It is recommended to use one of the ament license identitifers:
  Apache-2.0
  BSL-1.0
  BSD-2.0
  BSD-2-Clause
  BSD-3-Clause
  GPL-3.0-only
  LGPL-3.0-only
  MIT
  MIT-0

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

  ros2 run python_package_that_uses_the_interfaces amazing_quote_publisher_node 

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

  ros2 run python_package_that_uses_the_interfaces amazing_quote_publisher_node

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_interfaces/python_package_that_uses_the_interfaces/amazing_quote_publisher_node.py
   :language: python
   :linenos:
   :lines: 24-
   

