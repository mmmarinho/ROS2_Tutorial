.. include:: the_topic_is_under_heavy_construction.rst

Inspecting parameters (:program:`ros2 param`)
=============================================

ROS2 has a tool to interact with launch files called :program:`ros2 param`.

We can obtain more information on it with

.. code-block:: console

   ros2 param -h

which returns

.. code-block:: console
   :emphasize-lines: 11-15

   usage: ros2 param [-h] Call `ros2 param <command> -h` for more detailed usage. ...
  
   Various param related sub-commands
  
   options:
     -h, --help            show this help message and exit
  
   Commands:
     delete    Delete parameter
     describe  Show descriptive information about declared parameters
     dump      Dump the parameters of a node to a yaml file
     get       Get parameter
     list      Output a list of available parameters
     load      Load parameter file for a node
     set       Set parameter
  
     Call `ros2 param <command> -h` for more detailed usage.

As shown in the emphasized lines above, the :program:`ros2 param` tool has a large number of useful commands to interact with parameters.

Launching the Node with parameters
---------------------------------

.. hint::
   If you left the Node running from the last section, just keep it that way and skip this.

.. code-block:: console

   ros2 launch \
   python_package_that_uses_parameters_and_launch_files \
   peanut_butter_falcon_quote_publisher_launch.py

List-up parameters with :program:`ros2 param list`
--------------------------------------------------

.. hint::
   Remember that :ref:`Grep best friend`.

Similar to other ROS2 commands, we can get a list of currently loaded parameters with 

.. code-block:: console

   ros2 param list

which returns a well organized list showing the parameters of each active Node

.. code-block:: console

  /peanut_butter_falcon_quote_publisher_node:
    period
    philosopher_name
    quote
    topic_name
    use_sim_time

Obtain parameters with :program:`ros2 param get`
-----------------------------------------------

To obtain the value of a parameter, we can do as follows

.. code-block:: console

   ros2 param get \
   /peanut_butter_falcon_quote_publisher_node \
   quote

which will return the current value of the parameter, in this case, the initial value we set in the launch file

.. code-block:: console

   String value is: Yeah, you're gonna die, it's a matter of time. That ain't the question. The question's, whether they're gonna have a good story to tell about you when you're gone

Let's check the output of the Node
----------------------------------

.. hint::
   If you left :program:`ros2 topic echo` running from the last section, just keep it that way and skip this.

Before the next step, as we did in the past section, we do, **IN ANOTHER TERMINAL WINDOW**

.. code-block:: console

   ros2 topic echo /truly_inspirational_quote

Assign values to parameters with :program:`ros2 param set`
----------------------------------------------------------

For testing and regular usage, setting parameters from the command line is extremely helpful. Similar to how we are able to publish messages
to topics using a ROS2 tool, we can set a parameter with the following syntax

.. code-block:: console

   ros2 param set \
   /peanut_butter_falcon_quote_publisher_node \
   quote \
   "You got a good-guy heart. You can't do shit about it, that's just who you are. You're a hero." 

If everything is correct, we'll get

.. code-block:: console

   Set parameter successful

.. info::

   Some errors are easy to debug, such as when we get the name of the Node wrong

   .. code-block:: console
   
      Node not found

   but because of the interaction between the :program:`terminal`, :program:`ros2 param` itself, and the syntax of the services, its easy to find cryptic error messages.
   At first, always suppose that there's a typo somewhere.

Changing parameters is not instantaneous and, after the change becomes visible in the Node, our Node might have to loop once before it updates itself. We will be able to see that change as follows in the terminal window running :program:`ros2 topic echo`

.. code-block:: console

    id: 2220
    quote: Yeah, you're gonna die, it's a matter of time. That ain't the question. The question's, whether they're gonna have a good story ...
    philosopher_name: Tyler
    ---
    id: 2221
    quote: You got a good-guy heart. You can't do shit about it, that's just who you are. You're a hero.
    philosopher_name: Tyler
    ---
    id: 2222
    quote: You got a good-guy heart. You can't do shit about it, that's just who you are. You're a hero.
    philosopher_name: Tyler
    ---
    id: 2223
    quote: You got a good-guy heart. You can't do shit about it, that's just who you are. You're a hero.
    philosopher_name: Tyler
    ---
    id: 2224
    quote: You got a good-guy heart. You can't do shit about it, that's just who you are. You're a hero.
    philosopher_name: Tyler

Save parameters to a file with :program:`ros2 param dump`
---------------------------------------------------------

.. warning::

   Despite what the current :program:`ros2 param dump` `description <https://github.com/ros2/ros2cli/blob/86ae3930d4b56171ddff6d12cd467f6570ac6932/ros2param/ros2param/verb/dump.py#L40>`_ says, it does not actually dump the parameters to a file. It prints out the parameters in YAML file format to the terminal.

Words are sometimes little happy accidents. This usage of the word dump has no relation whatsoever to, for example, `Peter got dumped by Sarah and went to Hawaii <https://www.imdb.com/title/tt0800039/>`_. Dump files are usually related to `crashes and unresponsive programs <https://learn.microsoft.com/en-us/visualstudio/debugger/using-dump-files?view=vs-2022>`_, so this name puzzles me since ROS: the first.

While we wait for someone to come and correct me on my claims above, just think about this as a weird name for :program:`ros2 param print_to_screen_as_yaml`. It prints the parameters in the terminal with a YAML file format. It is nice because it gives a bit more info than :program:`ros2 param list`, but not so useful as-is. The trick is that we can put all that nicely formatted content into a file with

.. code-block:: console
   :emphasize-lines: 3

   ros2 param dump \
   /peanut_butter_falcon_quote_publisher_node \
   > peanut_butter_falcon_quote_publisher_node.yaml

where we are using the `>` to overwrite the contents of the :file:`peanut_butter_falcon_quote_publisher_node.yaml` file with the output of :program:`ros2 param dump`, so be careful not to overwrite your precious files by mistake.

We can inspect the contents of the file with

.. code-block:: console

   cat peanut_butter_falcon_quote_publisher_node.yaml

which outputs

.. code-block:: console

   /peanut_butter_falcon_quote_publisher_node:
     ros__parameters:
       period: 0.25
       philosopher_name: Tyler
       quote: Yeah, you're gonna die, it's a matter of time. That ain't the question.
         The question's, whether they're gonna have a good story to tell about you when
         you're gone
       topic_name: truly_inspirational_quote
       use_sim_time: false

