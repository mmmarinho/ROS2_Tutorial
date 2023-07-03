.. include:: the_topic_is_under_heavy_construction.rst

Inspecting parameters (:program:`ros2 param`)
=============================================

ROS2 has a tool to interact with launch files called :program:`ros2 param`.

We can obtain more information on it with

.. code-block:: console

   ros2 param -h

which returns

.. code-block:: console
   :emphasize-lines: 9,11-15

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

Lauching the Node with parameters
---------------------------------

.. hint::
   If you left the Node running from the last section, just keep it that way and skip this.

.. code-block:: console

   ros2 launch python_package_that_uses_parameters_and_launch_files peanut_butter_falcon_quote_publisher_launch.py

List-up parameters with :program:`ros2 param list`
--------------------------------------------------

.. hint::
   Remember that :ref:`Grep best friend`.

Similar to other ROS2 commands, we can get a list of currently loaded parameters with 

.. code-block:: console

   ros2 param list

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

which will return the current value of the parameter, in this case the initial value we set in the launch file

.. code-block:: console

   String value is: Yeah, you're gonna die, it's a matter of time. That ain't the question. The question's, whether they're gonna have a good story to tell about you when you're gone

Assign values to parameters with :program:`ros2 param set`
----------------------------------------------------------

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

Changing parameters is not instanteneous and, after the change becomes visible in the Node, our Node might have to loop once before it updates itself. We will be able to see that change as follows 

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

