Launch configurable Nodes (:program:`ros2 launch`)
--------------------------------------------------

ROS2 has a tool to interact with launch files called :program:`ros2 launch`.

We can obtain more information on it with

.. code-block:: console

   ros2 launch -h

which returns

.. code-block:: console

    usage: ros2 launch [-h] [-n] [-d] [-p | -s] [-a]
                       [--launch-prefix LAUNCH_PREFIX]
                       [--launch-prefix-filter LAUNCH_PREFIX_FILTER]
                       package_name [launch_file_name] [launch_arguments ...]
    
    Run a launch file
    
    positional arguments:
      package_name          Name of the ROS package which contains the launch
                            file
      launch_file_name      Name of the launch file
      launch_arguments      Arguments to the launch file; '<name>:=<value>' (for
                            duplicates, last one wins)
    
    options:
      -h, --help            show this help message and exit
      -n, --noninteractive  Run the launch system non-interactively, with no
                            terminal associated
      -d, --debug           Put the launch system in debug mode, provides more
                            verbose output.
      -p, --print, --print-description
                            Print the launch description to the console without
                            launching it.
      -s, --show-args, --show-arguments
                            Show arguments that may be given to the launch file.
      -a, --show-all-subprocesses-output
                            Show all launched subprocesses' output by overriding
                            their output configuration using the
                            OVERRIDE_LAUNCH_PROCESS_OUTPUT envvar.
      --launch-prefix LAUNCH_PREFIX
                            Prefix command, which should go before all
                            executables. Command must be wrapped in quotes if it
                            contains spaces (e.g. --launch-prefix 'xterm -e gdb
                            -ex run --args').
      --launch-prefix-filter LAUNCH_PREFIX_FILTER
                            Regex pattern for filtering which executables the
                            --launch-prefix is applied to by matching the
                            executable name.

Despite the large number of possible options, there are no notable examples of options that are of particular use to us right now.

We can call our Node, configured with our launch file, with

.. code-block:: console

   ros2 launch python_package_that_uses_parameters_and_launch_files peanut_butter_falcon_quote_publisher_launch.py

which returns

.. code-block:: console
  
    [INFO] [launch]: All log files can be found below /home/murilo/.ros/log/2023-06-30-17-00-07-522194-murilos-toaster-2963
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [amazing_quote_configurable_publisher_node-1]: process started with pid [2964]

showing that the launch was successful.

**IN ANOTHER TERMINAL** we run

.. code-block:: console

   ros2 topic echo /truly_inspirational_quote

resulting in something similar to

.. code-block:: console
    
    id: 301
    quote: Yeah, you're gonna die, it's a matter of time. That ain't the question. The question's, whether they're gonna have a good story ...
    philosopher_name: Tyler
    ---
    id: 302
    quote: Yeah, you're gonna die, it's a matter of time. That ain't the question. The question's, whether they're gonna have a good story ...
    philosopher_name: Tyler
    ---
    id: 303
    quote: Yeah, you're gonna die, it's a matter of time. That ain't the question. The question's, whether they're gonna have a good story ...
    philosopher_name: Tyler
    ---

And there you have it. Feeling inspired yet?
