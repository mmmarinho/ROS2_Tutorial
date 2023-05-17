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
   This API `might change <https://github.com/ros2/rclpy/issues/464>`_, but for now we have to roll with it.

:download:`what_is_the_point_service_server_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 1


Create the Node with a Service Client
-------------------------------------

:download:`what_is_the_point_service_client_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 24-
   :emphasize-lines: 1

Update the :file:`setup.py`
---------------------------

As we already learned in :ref:`Making rosrun work`, we must adjust the :file:`setup.py` to refer to the Nodes we just created.

:download:`setup.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/setup.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/setup.py
   :language: python
   :linenos:
   :emphasize-lines: 21-25

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: the_canonical_build_command.rst

Testing Service Server and Client
---------------------------------

.. code:: console

   ros2 run python_package_that_uses_the_services what_is_the_point_service_client_node

when running the client Node, the server is still not active. In that case, the client node will keep waiting for it, as follows

.. code:: console

    [INFO] [1684293008.888276849] [what_is_the_point_service_client]: service /what_is_the_point not available, waiting...
    [INFO] [1684293009.890589539] [what_is_the_point_service_client]: service /what_is_the_point not available, waiting...
    [INFO] [1684293010.892778194] [what_is_the_point_service_client]: service /what_is_the_point not available, waiting...

In another terminal, we run the :program:`python_package_uses_the_service_node`, as follows

.. code:: console

    ros2 run python_package_that_uses_the_services what_is_the_point_service_server_node

the server Node will then output, periodically,

.. code:: console

    [INFO] [1684305461.723440894] [what_is_the_point_service_server]:
                Valued Customer,

                Thank you for calling the `WhatIsThePoint` service callback helpline.
                We are proud to have been called 3 times.

                **-**-**-**-**-**-**-**-**-**-**-**-

                We have analysed the following AmazingQuote:

                [...] your living... it is always potatoes. I dream of potatoes.

                -- a young Maltese potato farmer

                The point has been sent to you accordingly.
                Feel free to check the response at your convenience.

                **-**-**-**-**-**-**-**-**-**-**-**-

                Have a nice day,
                Service Callerson Jr.


and the client Node will output, periodically,

.. code:: console

    [INFO] [1684305461.224588634] [what_is_the_point_service_client]:
                    #$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$

                    We have thus received the point of our quote.

                                (21.480765227729144, 1.8148233711302457, 18.70441140114061)

                    #$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$#$

