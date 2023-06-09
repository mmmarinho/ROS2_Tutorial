Inspecting services (:program:`ros2 service`)
=============================================

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

Start a service server
----------------------

Similar to the discussion about topics, it is good to be able to test service servers without having to develop a complete service client. Let's start by running the service server we created just now.

.. warning::

   Be sure to terminate the Nodes we used in the past section before proceeding (e.g. with :kbd:`CTRL+C`), otherwise, the output will look different from what is described here.

.. code:: console

  ros2 run python_package_that_uses_the_services what_is_the_point_service_server_node

Getting all services with :program:`ros2 service list`
------------------------------------------------------

To see all currently active services, we run

.. code:: console

   ros2 service list
   
which, in this case, outputs

.. code-block:: console
    :emphasize-lines: 1

    /what_is_the_point
    /what_is_the_point_service_server/describe_parameters
    /what_is_the_point_service_server/get_parameter_types
    /what_is_the_point_service_server/get_parameters
    /what_is_the_point_service_server/list_parameters
    /what_is_the_point_service_server/set_parameters
    /what_is_the_point_service_server/set_parameters_atomically

To everyone's surprise, there are a lot of services beyond the one we created. We can address those when we talk about ROS2 parameters, for now, we ignore them.

Testing your service servers with ros2 service call
---------------------------------------------------

Like the discussion about topics, ROS2 has a tool to call a service from the terminal, called :program:`ros2 service call`. The service must be specified and an instance of its request must be written using :abbr:`YAML (YAML Ain't Markup Language)`.
Back to our example, we can do

.. code-block:: console

    ros2 service call /what_is_the_point \
    package_with_interfaces/srv/WhatIsThePoint \
    '{ 
    quote: {
         id: 1994,
         quote: So you’re telling me there’s a chance, 
         philosopher_name: Lloyd 
         }
    }'

which results in

.. code-block:: console
    
    waiting for service to become available...
    requester: making request: package_with_interfaces.srv.WhatIsThePoint_Request(quote=package_with_interfaces.msg.AmazingQuote(id=1994, quote='So you’re telling me there’s a chance', philosopher_name='Lloyd'))
    
    response:
    package_with_interfaces.srv.WhatIsThePoint_Response(point=geometry_msgs.msg.Point(x=8.327048266159165, y=95.97987946924988, z=67.03878311627777))

Testing your service clients???
-------------------------------

To the best of my knowledge, there is no tool inside :program:`ros2 service` to allow us to experiment with the service clients. For service clients, apparently, the only way to test them is to make a minimal service server to interact with them. We've already done that, so this topic ends here.
