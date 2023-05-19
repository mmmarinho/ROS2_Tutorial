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

The Node
^^^^^^^^

This example deviates somewhat from what is done in the `official examples <https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_client/examples_rclpy_minimal_client>`_.
This implementation shown herein uses a callback and :code:`rclpy.spin()`.
It has many practical but it's no *panacea*.

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

Instantiate a :code:`Future` as class attribute
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

Instantiate a Timer
^^^^^^^^^^^^^^^^^^^

Whenever periodic work must be done, it is recommended to use a :code:`Timer`, as we already learned in :ref:`Use a Timer for periodic work`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 49-52
   :lineno-start: 26

The need for a callback for the :code:`Timer`, should also be no surprise.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 54-55
   :lineno-start: 31

Service Clients use :code:`<srv>.Request()`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Given that services work in a request--response model, the service client must instantiate a suitable :code:`<srv>.Request()` and populate its fields before making the service call.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 57-65
   :lineno-start: 34

Make service calls with :code:`call_async()`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The recommended way to initiate service calls is through :code:`call_async()`, which is the reason why we are working with :code:`async` logic. In general, the result of the call, a :code:`Future`, will still not have the result of the service call. 

There are many ways to address the use of a :code:`Future`. One of them, especially tailored for interfacing :code:`async` with callback-based frameworks is the :code:`Future.add_done_callback()`. If the :code:`Future` is already done by the time we call :code:`add_done_callback()`, it is supposed to `call the callback for us <https://github.com/ros2/rclpy/blob/0f1af0db16c38899aaea1fb1ca696800255d2b55/rclpy/rclpy/task.py#L163>`_.

The benefit of this is that the callback will not hold our resources until the response is ready. When the response is ready, and the executor gets to processing callbacks, our callback will be called *automagically*.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 67-72
   :lineno-start: 44
   :emphasize-lines: 5,6

Given that we are periodically calling the service, before we change the value of the :code:`Future`, we can check if the service call was done with :code:`Future.done()`. If it is not done, we can use :code:`Future.cancel()` so that our callback can handle this case as well. For instance, if the Service Server has been shutdown, the :code:`Future` would never be done.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 67-72
   :lineno-start: 44
   :emphasize-lines: 1-4

The Future callback
^^^^^^^^^^^^^^^^^^^

The callback for the :code:`Future` must receive a :code:`Future` as argument. Having it as an attribute of the Node's class allows us to access ROS2 method such as :code:`get_logger()` and other contextual information.

The result of the :code:`Future` is obtained using :code:`Future.result()`. The response might be :code:`None` in some cases, so we must check it before trying to use the result, otherwise we will get a nasty exception.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 74-88
   :lineno-start: 51
   :emphasize-lines: 1,3,4

Update the :file:`setup.py`
---------------------------

As we already learned in :ref:`Making rosrun work`, we must adjust the :file:`setup.py` to refer to the Nodes we just created.

:download:`setup.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/setup.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/setup.py
   :language: python
   :linenos:
   :emphasize-lines: 23-26

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

