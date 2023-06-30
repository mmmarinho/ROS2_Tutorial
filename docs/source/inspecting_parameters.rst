.. include:: the_topic_is_under_heavy_construction.rst

Inspecting parameters (:program:`ros2 param`)
=============================================

.. code-block:: console

   ros2 param -h

.. code-block:: console

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

.. code-block:: console

   ros2 param list

.. code-block:: console

  /peanut_butter_falcon_quote_publisher_node:
    period
    philosopher_name
    quote
    topic_name
    use_sim_time

.. code-block:: console

   ros2 param get /peanut_butter_falcon_quote_publisher_node quote

.. code-block:: console

   String value is: Yeah, you're gonna die, it's a matter of time. That ain't the question. The question's, whether they're gonna have a good story to tell about you when you're gone

.. code-block:: console

   ros2 param set /peanut_butter_falcon_quote_publisher_node quote "You got a good-guy heart. You can't do shit about it, that's just who you are. You're a hero." 
Set parameter successful

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

