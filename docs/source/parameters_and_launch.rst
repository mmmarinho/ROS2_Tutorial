.. include:: the_topic_is_under_heavy_construction.rst

Parameters: Launching configurable nodes
========================================

The Nodes we have made in the past few sections are interesting because they take advantage of the interprocess communication provided by ROS2. 

Other capabilities of ROS2 that we must take advantage of are `ROS2 parameters <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html>`_ and `ROS2 launch files <https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html>`_. We can use them to modify the behavior of Nodes without having to modify their source code. For Python users, that might sound less appealing than users of compiled languages. However, users of your package might not want nor be able to modify the source code directly, if the package is installable or part of a larger system with multiple users.

  
Create the package
------------------

First, let us create an :program:`ament_python` package that depends on our :file:`packages_with_interfaces` and build from there.

.. code:: console

  cd ~/ros2_tutorial_workspace/src
  ros2 pkg create python_package_that_uses_parameters_and_launch_files \
  --build-type ament_python \
  --dependencies rclpy package_with_interfaces


Overview
--------

Before we start exploring the elements of the package, let us

#. Create the Node with a configurable publisher using parameters, mostly as we saw in :ref:`Create a publisher`.
#. Create a launch file to configure the Node without modifying its source code.

Create the Node using parameters
--------------------------------

.. admonition:: **TL:DR** Using parameters in a Node

               #. Declare the parameter with :code:`Node.declare_parameter()`, usually in the class's :code:`__init__`.
               #. Get the parameter with :code:`Node.get_parameter()` either once or continuously.

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 4
      
      src/python_package_that_uses_parameters_and_launch_files
        └── python_package_that_uses_parameters_and_launch_files/
              └── __init__.py
              └── amazing_quote_configurable_publisher_node.py

For the sake of the example, let us suppose that we want to make an :code:`AmazingQuote` publisher that is, now, configurable.

Let's start by creating an :file:`amazing_quote_configurable_publisher_node.py` in :file:`python_package_that_uses_parameters_and_launch_files/python_package_that_uses_parameters_and_launch_files` with the following contents

:download:`amazing_quote_configurable_publisher_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/python_package_that_uses_parameters_and_launch_files/amazing_quote_configurable_publisher_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/python_package_that_uses_parameters_and_launch_files/amazing_quote_configurable_publisher_node.py
   :language: python
   :lines: 24-
   :linenos:

Don't forget to declare the parameter!
--------------------------------------

It's easy to forget it, but :code:`Node.get_parameter()` will not work if the parameter was not first declared with :code:`Node.declare_parameter()`. Don't forget it!

One-off parameters
------------------

.. hint:

   The second parameter of :code:`declare_parameter()` is the default value of a parameter. It can be used to give a value when a parameter is not externally defined.
   If you want to enforce that this parameter must be defined, do not define a second argument when calling the method.

For one-off parameters, we just get them once after declaring them. Because we're using those attributes directly in the :code:`__init__` method, they are not made attributes of the class, but they could be.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/python_package_that_uses_parameters_and_launch_files/amazing_quote_configurable_publisher_node.py
   :language: python
   :lines: 39-50
   :emphasize-lines: 2-5,8,9,12

In this case, we're making the topic name and publication periodicity as one-off configurable parameters. 

Continuously-obtained parameters
--------------------------------

For parameters that we obtain continuously through the lifetime of the Node, we can, for example, declare them in the :code:`__init__` method, like so

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/python_package_that_uses_parameters_and_launch_files/amazing_quote_configurable_publisher_node.py
   :language: python
   :lines: 35-37

then obtain them in another method, like so

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/python_package_that_uses_parameters_and_launch_files/amazing_quote_configurable_publisher_node.py
   :language: python
   :lines: 54-67
   :emphasize-lines: 4,5,9,10

In this example, we are making the :code:`quote` and the :code:`philosopher_name` as configurable parameters that can be changed continuously, during the lifetime of the Node. After they are changed, the node will publish a message with different contents.

Truly configurable: using :file:`_launch.py` files
--------------------------------------------------

.. admonition:: **TL:DR** Using launch files

               #. (Once) Create a :file:`launch` folder in the project.
               #. Create the launch file named as :file:`launch/<something>_launch.py`.
               #. (Once) modify the :file:`setup.py` to correctly install launch files.

.. note:
   For a previous user of ROS1 used with the :abbr:`XML (Extensible Markup Language)`\ -based :file:`.launch` files, switching for the Python-based ones is a hassle.
   However, my experience with these so far has been quite positive, because when using Python we have access to an entire ecosystem of tools to make the launch files
   smarter, whereas with the :abbr:`XML (Extensible Markup Language)`\ -based ones, if possible at all, we had to add hack on top of hack to achieve the same. 

Differently from ROS1, in ROS2 we can use Python launch files. They are quite powerful, well documented, and mentioned first `in the official documentation <https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html>`_, so we will use them instead of :abbr:`XML (Extensible Markup Language)` or :abbr:`YAML (YAML ain't markup language)` files.

(Once) create the :file:`launch` folder
---------------------------------------

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 5
      
      src/python_package_that_uses_parameters_and_launch_files
        └── python_package_that_uses_parameters_and_launch_files/
              └── __init__.py
              └── amazing_quote_configurable_publisher_node.py
        └── launch

Well, without further ado

.. code:: console

  cd ~/ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files
  mkdir launch

Create the :file:`launch` file
------------------------------

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 6
      
      src/python_package_that_uses_parameters_and_launch_files
        └── python_package_that_uses_parameters_and_launch_files/
              └── __init__.py
              └── amazing_quote_configurable_publisher_node.py
        └── launch
              └── peanut_butter_falcon_quote_publisher_launch.py

.. warning:
   The Python launch file **MUST** have the suffix :file:`_launch.py`. It will be used by the :file:`setup.py` to install it correctly.

Suppose that we are tired of all the meme quotes and want to make our Node publish a truly inspirational quote. We start by making the launch file named :file:`peanut_butter_falcon_quote_publisher_launch.py` within the :file:`launch` folder we just created, with the following contents

:download:`peanut_butter_falcon_quote_publisher_launch.py <../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/launch/peanut_butter_falcon_quote_publisher_launch.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/launch/peanut_butter_falcon_quote_publisher_launch.py
   :language: python
   :linenos: 

We're relying on the :code:`LaunchDescription`, which expects a list of :code:`launch_ros.actions`. 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/launch/peanut_butter_falcon_quote_publisher_launch.py
   :language: python
   :lines: 1,2

When using a :code:`launch_ros.actions.Node`, we need to define which :file:`package` it belongs to and the :file:`executable` which must match the name
we set for the executable in the :file:`setup.py`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/launch/peanut_butter_falcon_quote_publisher_launch.py
   :language: python
   :lines: 8,9

Besides the parameters, we can configure the name of the Node, such that each is unique

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/launch/peanut_butter_falcon_quote_publisher_launch.py
   :language: python
   :lines: 10

Finally, our parameters are defined using a dictionary within a list, namely

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/launch/peanut_butter_falcon_quote_publisher_launch.py
   :language: python
   :lines: 12-16

The :file:`setup.py`
--------------------

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 7
      
      src/python_package_that_uses_parameters_and_launch_files
        └── python_package_that_uses_parameters_and_launch_files/
              └── __init__.py
              └── amazing_quote_configurable_publisher_node.py
        └── launch
              └── peanut_butter_falcon_quote_publisher_launch.py
        setup.py

Modify the :file:`setup.py` to look like this

:download:`setup.py <../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/setup.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_parameters_and_launch_files/setup.py
   :language: python
   :linenos:
   :emphasize-lines: 15,25-27

We have already seen a :file:`setup.py` so many times we're almost calling it `Wilson <https://www.imdb.com/name/nm1012434/>`_. 
The only difference is emphasized above inside the :code:`data_files`, which is the line that will specify that launch files will be installed as well. Notice that
the :file:`setup.py` looks for files with a specific pattern in the folder :file:`launch`, so be sure that your launch files
have the correct name otherwise they might not be installed as expected.

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: the_canonical_build_command.rst

Launch files to the moon with :program:`ros2 launch`
----------------------------------------------------

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

