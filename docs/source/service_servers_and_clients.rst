At your Service: Servers and Clients
====================================

.. note::

   Except for the particulars of the :file:`setup.py` file, the way that services in ROS2 work in Python, i.e. the explanation in this section, does not depend on :program:`ament_python` or :program:`ament_cmake`.


In some cases, we need means of communication in which each command has an associated response. That is where :code:`Services` come into play.

Create the package
------------------

We start by creating a package to use the :code:`Service` we first created in :ref:`The service file`.

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
         #. Import new services :code:`from <package_name>.srv import <srv_name>`
         #. In a subclass of :code:`Node`

            #. create a callback :code:`def callback(self, request, response):`
            #. create a service server with :code:`self.service_server = self.create_service(...)`

         #. Add the new Node to :file:`setup.py`

:download:`what_is_the_point_service_server_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :linenos:
   :lines: 24-

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :linenos:
   :lines: 24-29
   :emphasize-lines: 6

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :linenos:
   :lines: 35-43
   :emphasize-lines: 4-7

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :linenos:
   :lines: 45-49

.. note::

   The API for the Service Server callback is a bit weird in that it receives the response as an argument.
   This API `might change <https://github.com/ros2/rclpy/issues/464>`_, but for now this is what we got.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :linenos:
   :lines: 64-69

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :linenos:
   :lines: 88-90
   :emphasize-lines: 2

Service Clients
---------------

ROS2 :code:`rclpy` Service Clients are implemented using an :code:`asyncio` logic (`More info <https://docs.python.org/3.10/library/asyncio.html>`_).
In this tutorial, we briefly introduce unavoidable :code:`async` concepts, but for any extra understanding it's better to check the official documentation.

Create the Node with a Service Client (using a :code:`callback`)
----------------------------------------------------------------

.. note::

         **TL:DR** Creating a service client (using a :code:`callback`)

         #. Add new dependencies to :file:`package.xml`
         #. Import new services :code:`from <package_name>.srv import <srv_name>`
         #. In a subclass of :code:`Node`

            #. (*recommended*) wait for service to be available :code:`service_client.wait_for_service(...)`.
            #. (*if periodic*) add a :code:`Timer` with a proper :code:`timer_callback()`
            #. create a callback for the future :code:`def service_future_callback(self, future: Future):`
            #. create a service client with :code:`self.service_client = self.create_client(...)`

         #. Add the new Node to :file:`setup.py`

.. note::
   This example deviates somewhat from what is done in the `official examples <https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_client/examples_rclpy_minimal_client>`_.
   This implementation shown herein uses a callback and :code:`rclpy.spin()`.
   It has many practical but it's no *panacea*.

The Node
^^^^^^^^

:download:`what_is_the_point_service_client_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 24-

Imports
^^^^^^^^

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 31
   :lineno-start: 8

Instantiate a Service Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:code:`create_client`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 40-42
   :lineno-start: 17

(Recommended) Wait for the Service Server to be available
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:code:`wait_for_service`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 44,45
   :lineno-start: 21

(Recommended) Instantiate a :code:`Future` as class attribute
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:code:`Future` (`More info <https://docs.python.org/3.10/library/asyncio-future.html#asyncio-futures>`_)

.. note::
   Asynchronous code is not the same as code that runs in parallel, even more so in Python because of the :abbr:`GIL (Global Interpreter Lock)` (`More info <https://wiki.python.org/moin/GlobalInterpreterLock>`_).
   Basically, the :code:`async` framework allows us to not waste time waiting for results that we don't know when will arrive.
   It either allows us to attach a :code:`callback` for when the result is ready, or to run many service calls and :code:`await`
   for them all, instead of running one at a time.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 47
   :lineno-start: 24

.. note::

   If the :code:`Future` is already done by the time we call :code:`add_done_callback()`, it is supposed to `call the callback for us <https://github.com/ros2/rclpy/blob/0f1af0db16c38899aaea1fb1ca696800255d2b55/rclpy/rclpy/task.py#L163>`_.

(If periodic) Instatiate a Timer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Whenever periodic work must be done, it is recommended to use a :code:`Timer`, as we already learned in :ref:`Use a Timer for periodic work`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 39-42
   :lineno-start: 26

Update the :file:`setup.py`
---------------------------

As we already learned in :ref:`Making rosrun work`, we must adjust the :file:`setup.py` to refer to the Nodes we just created.

:download:`setup.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/setup.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/setup.py
   :language: python
   :linenos:
   :emphasize-lines: 22-26

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

