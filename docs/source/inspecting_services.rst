Inspecting services (:program:`ros2 service`)
=============================================

.. include:: the_topic_is_under_heavy_construction.rst

ROS2 has a tool to help us inspect services. It is just as helpful as the tools for topics.

.. code:: console

    ros2 service -h

which outputs the detailed information of the tool, as shown below. In particular, the highlighted fields are used quite frequently in practice.

.. code-block:: console
    :emphasize-lines: 13,15

    usage: ros2 service [-h] [--include-hidden-services]
                        Call `ros2 service <command> -h` for more
                        detailed usage. ...
    
    Various service related sub-commands
    
    options:
      -h, --help            show this help message and exit
      --include-hidden-services
                            Consider hidden services as well
    
    Commands:
      call  Call a service
      find  Output a list of available services of a given type
      list  Output a list of available services
      type  Output a service's type
    
      Call `ros2 service <command> -h` for more detailed usage.
