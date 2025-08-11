Inspecting services (:program:`ros2 service`)
=============================================

:program:`ROS2` has a tool to help us inspect services. It is just as helpful as the tools for topics.

.. code:: console

    ros2 service -h

which outputs the detailed information of the tool, as shown below. In particular, the highlighted fields are used quite frequently in practice.

.. code-block:: console
    :emphasize-lines: 11,15

    usage: ros2 service [-h] [--include-hidden-services] Call `ros2 service <command> -h` for more detailed usage. ...

    Various service related sub-commands

    options:
      -h, --help            show this help message and exit
      --include-hidden-services
                            Consider hidden services as well

    Commands:
      call  Call a service
      echo  Echo a service
      find  Output a list of available services of a given type
      info  Print information about a service
      list  Output a list of available services
      type  Output a service's type

      Call `ros2 service <command> -h` for more detailed usage.

Start a service server
----------------------

Similar to the discussion about topics, it is good to be able to test service servers without having to develop a complete service client. Let's start by running the service server we created just now.

.. warning::

   Be sure to terminate the Nodes we used in the past section before proceeding (e.g. with :kbd:`CTRL+C`), otherwise, the output will look different from what is described here.

.. code:: console

  ros2 run python_package_that_uses_the_services add_points_service_server_node

Getting all services with :program:`ros2 service list`
------------------------------------------------------

To see all currently active services, we run

.. code:: console

   ros2 service list
   
which, in this case, outputs

.. code-block:: console
    :emphasize-lines: 1

    /add_points
    /add_points_service_server/describe_parameters
    /add_points_service_server/get_parameter_types
    /add_points_service_server/get_parameters
    /add_points_service_server/get_type_description
    /add_points_service_server/list_parameters
    /add_points_service_server/set_parameters
    /add_points_service_server/set_parameters_atomically

To everyone's surprise, there are a lot of services beyond the one we created. We can address those when we talk about ROS2 parameters, for now, we ignore them.

Testing your service servers with ros2 service call
---------------------------------------------------

Like the discussion about topics, ROS2 has a tool to call a service from the terminal, called :program:`ros2 service call`. The service must be specified and an instance of its request must be written using :abbr:`YAML (YAML Ain't Markup Language)`.
Back to our example, we can do

.. code-block:: console

    ros2 service call /add_points \
    package_with_interfaces/srv/AddPoints \
    '{ 
    a: {
         x: 10,
         y: 11,
         z: 12
         },
    b: {
         x: -10,
         y: -10,
         z: 22
       }
    }'

which results in

.. code-block:: console
    
    requester: making request: package_with_interfaces.srv.AddPoints_Request(a=geometry_msgs.msg.Point(x=10.0, y=11.0, z=12.0), b=geometry_msgs.msg.Point(x=-10.0, y=-10.0, z=22.0))

    response:
    package_with_interfaces.srv.AddPoints_Response(result=geometry_msgs.msg.Point(x=0.0, y=1.0, z=34.0))

Testing your service clients???
-------------------------------

To the best of my knowledge, there is no tool inside :program:`ros2 service` to allow us to experiment with the service clients. For service clients, apparently, the only way to test them is to make a minimal service server to interact with them. We've already done that, so this topic ends here.

How about :program:`ros2 service echo`
--------------------------------------

.. seealso::

   - https://docs.ros.org/en/jazzy/Tutorials/Demos/Service-Introspection.html
   - https://github.com/ros2/demos/blob/rolling/demo_nodes_py/demo_nodes_py/services/introspection.py


It is important to know of the existence of ``service introspection``. Although it seems to work the documentation might be ongoing https://github.com/ros-infrastructure/rep/pull/360>_

After proper configuration of the server and client, :program:`ros2 service echo` allows us to look at what information is being exchanged between participants. It helps not having to
add endless ``print`` functions throughout the code that hurt performance.

We'll be working in these files

.. code-block:: console

    python_package_that_uses_the_services
    `-- python_package_that_uses_the_services
        |-- add_points_service_client_introspection_node.py
        `-- add_points_service_server_introspection_node.py


.. tab-set::

    .. tab-item:: Introspection Client

        :download:`add_points_service_client_introspection_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_introspection_node.py>`

        .. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_introspection_node.py
           :language: python
           :linenos:
           :lines: 24-

    .. tab-item:: Introspection Server

        :download:`add_points_service_server_introspection_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_server_introspection_node.py>`

        .. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_server_introspection_node.py
           :language: python
           :linenos:
           :lines: 24-
