At your Service: Servers and Clients
====================================

.. note::

   Except for the particulars of the :file:`setup.py` file, the way that services in ROS2 work in Python, i.e. the explanation in this section, does not depend on :program:`ament_python` or :program:`ament_cmake`.


In some cases, we need means of communication in which each command has an associated response. That is where :code:`Services` come into play.

Create the package
------------------

We start by creating a package to use the :code:`Service` we first created in :ref:`Services`.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_the_services \
    --build-type ament_python \
    --dependencies rclpy package_with_interfaces

Overview
--------

Before we start exploring the elements of the package, let us

#. Create the Node with a Service Server.
#. Create the Node with a Service Client.
#. Update the :file:`setup.py` so that :program:`ros2 run` finds these programs.

.. _Create a service server:

Create the Node with a Service Server
-------------------------------------

.. note::

         **TL:DR** Creating a service server

         #. Add new dependencies to :file:`package.xml`
         #. Import new services :code:`from <package_name>.srv import <msg_name>`
         #. In a subclass of :code:`Node`

            #. create a callback :code:`def callback(self, request, response):`
            #. create a publisher with :code:`self.publisher = self.create_service(...)`

         #. Add the new Node to :file:`setup.py`

.. note::

   The API for the Service Server callback is a bit weird in that it receives the response as an argument.
   This api `might change <https://github.com/ros2/rclpy/issues/464>`_, but for now we have to roll with it.

:download:`what_is_the_point_service_server.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 1


Create the Node with a Service Client
-------------------------------------

:download:`what_is_the_point_service_client.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 1


.. code:: console

   ros2 run python_package_that_uses_the_services what_is_the_point_service_client_node


.. code:: console

    [INFO] [1684293008.888276849] [what_is_the_point_service_client]: service /what_is_the_point not available, waiting...
    [INFO] [1684293009.890589539] [what_is_the_point_service_client]: service /what_is_the_point not available, waiting...
    [INFO] [1684293010.892778194] [what_is_the_point_service_client]: service /what_is_the_point not available, waiting...

.. code:: console

    ros2 run python_package_that_uses_the_services what_is_the_point_service_server_node

.. code:: console

    [INFO] [1684293071.140372053] [what_is_the_point_service_server]:
                Valued Customer,

                Thank you for calling the `WhatIsThePoint` service callback helpline.
                We are proud to have been called 1 times.

                **-**-**-**-**-**-**-**-**-**-**-**-

                We have analysed the following AmazingQuote:

                My life, it's potato...
                                         In your working life, and your living
                                         it is always potatoes. I dream of potatoes.

                -- a young Maltese potato farmer

                The point has been sent to you accordingly.
                Feel free to check the response at your convenience.

                **-**-**-**-**-**-**-**-**-**-**-**-

                Have a nice day,
                Service Callerson Jr.
