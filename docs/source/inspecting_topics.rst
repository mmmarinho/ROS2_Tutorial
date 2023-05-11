Inspecting topics (:program:`ros2 topic`)
=========================================

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
