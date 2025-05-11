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

.. admonition::  **TL;DR** Creating a service server
      
               #. Add new dependencies to :file:`package.xml`
               #. Import new services :code:`from <package_name>.srv import <srv_name>`
               #. In a subclass of :code:`Node`
      
                  #. create a callback :code:`def callback(self, request, response):`
                  #. create a service server with :code:`self.service_server = self.create_service(...)`
      
               #. Add the new Node to :file:`setup.py`

Let's start by creating a :file:`what_is_the_point_service_server_node.py` in :file:`~/ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services` with the following contents

:download:`what_is_the_point_service_server_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :lines: 24-

The code begins with an import to the service we created. No surprise here.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :lines: 24-29
   :emphasize-lines: 6

The Service Server must be initialised with the :code:`create_service()`, as follows, with parameters that should by now be quite obvious to us.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :lines: 38-41

The Service Server receives a :code:`WhatIsThePoint.Request` and returns a :code:`WhatIsThePoint.Response`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :lines: 45-52
   :emphasize-lines: 2,4

.. warning::

   The API for the Service Server callback is a bit weird in that needs the :code:`Response` as an argument.
   This API `might change <https://github.com/ros2/rclpy/issues/464>`_, but for now this is what we got.

We play around with the :code:`WhatIsThePoint.Request` a bit and use that result to populate a :code:`WhatIsThePoint.Response`, as follows

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :lines: 66-69

At the end of the callback, we must return that :code:`WhatIsThePoint.Request`, like so 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_server_node.py
   :language: python
   :lines: 85
   :emphasize-lines: 2

The Service Server was quite painless, but it doesn't do much. The Service Client might be a bit more on the painful side for the uninitiated.

Service Clients
---------------

ROS2 :code:`rclpy` Service Clients are implemented using an :code:`asyncio` logic (`More info <https://docs.python.org/3.10/library/asyncio.html>`_).
In this tutorial, we briefly introduce unavoidable :code:`async` concepts in :ref:`Asyncio`, but for any extra understanding, it's better to check the official documentation.

Create the Node with a Service Client (using a :code:`callback`)
----------------------------------------------------------------

.. admonition::  **TL;DR** Creating a Service Client (using a :code:`callback`)

               #. Add new dependencies to :file:`package.xml`
               #. Import new services :code:`from <package_name>.srv import <srv_name>`
               #. In a subclass of :code:`Node`
      
                  #. (*recommended*) wait for service to be available :code:`service_client.wait_for_service(...)`.
                  #. (*if periodic*) add a :code:`Timer` with a proper :code:`timer_callback()`
                  #. create a callback for the future :code:`def service_future_callback(self, future: Future):`
                  #. create a Service Client with :code:`self.service_client = self.create_client(...)`
      
               #. Add the new Node to :file:`setup.py`

The Node
^^^^^^^^

.. note::
   This example deviates somewhat from what is done in the `official examples <https://github.com/ros2/examples/tree/humble/rclpy/services/minimal_client/examples_rclpy_minimal_client>`_.
   This implementation shown herein uses a callback and :code:`rclpy.spin()`.
   It has many practical applications, but it's no *panacea*.

We start by adding a :file:`what_is_the_point_service_client_node.py` at :file:`python_package_that_uses_the_services/python_package_that_uses_the_services` with the following contents.

:download:`what_is_the_point_service_client_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :linenos:
   :lines: 24-

Imports
^^^^^^^

To have access to the service, we import it with :code:`from <package>.srv import <Service>`. 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 31

Instantiate a Service Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We instantiate a Service Client with :code:`Node.create_client()`. The values of :code:`srv_type` and :code:`srv_name` must match the ones used in the Service Server.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 40-42

(Recommended) Wait for the Service Server to be available
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
   The order of execution and speed of Nodes depend on a complicated web of relationships between ROS2, the operating system, and the workload of the machine. It would be naive to expect the server to always be active before the client, even if the server Node is started before the client Node.

In many cases, having the result of the service is of particular importance (hence the use of a service and not messages). In that case, we have to wait until :code:`service_client.wait_for_service()`, as shown below.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 44,45

Instantiate a :code:`Future` as a class attribute
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As part of the :code:`async` framework, we instantiate a :code:`Future` (`More info <https://docs.python.org/3.10/library/asyncio-future.html#asyncio-futures>`_). In this example it is important to have it as an attribute of the class so that we do not lose the reference to it after the callback.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 47

Instantiate a Timer
^^^^^^^^^^^^^^^^^^^

Whenever periodic work must be done, it is recommended to use a :code:`Timer`, as we already learned in :ref:`Use a Timer for periodic work`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 49-52

The need for a callback for the :code:`Timer`, should also be no surprise.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 54-55

Service Clients use :code:`<srv>.Request()`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Given that services work in a request-response model, the Service Client must instantiate a suitable :code:`<srv>.Request()` and populate its fields before making the service call, as shown below. To make the example more interesting, it randomly switches between two possible quotes.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 57-65

Make service calls with :code:`call_async()`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :code:`async` framework in ROS2 is based on Python's :code:`asyncio` that we already saw in :ref:`Asyncio`.
   
.. note::
   At first glance, it might feel that all this trouble to use :code:`async` is unjustified. However, Nodes in practice will hardly ever do one service call and be done. Many Nodes in a complex system
   will have a composition of many service servers, service clients, publishers, and subscribers. Blocking the entire Node while it waits for the result of a service is, in most cases, a bad design.

The recommended way to call a service is through :code:`call_async()`, which is the reason why we are working with :code:`async` logic. In general, the result of :code:`call_async()`, a :code:`Future`, will not have the result of the service call at the next line of our program. 

There are many ways to address the use of a :code:`Future`. One of them, specially tailored to interface :code:`async` with callback-based frameworks is the :code:`Future.add_done_callback()`. If the :code:`Future` is already done by the time we call :code:`add_done_callback()`, it is supposed to `call the callback for us <https://github.com/ros2/rclpy/blob/0f1af0db16c38899aaea1fb1ca696800255d2b55/rclpy/rclpy/task.py#L163>`_.

The benefit of this is that the callback will not block our resources until the response is ready. When the response is ready, and the ROS2 executor gets to processing :code:`Future` callbacks, our callback will be called *automagically*.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 67-72
   :emphasize-lines: 5,6

Given that we are periodically calling the service, before replace the class :code:`Future` with the next service call, we can check if the service call was done with :code:`Future.done()`. If it is not done, we can use :code:`Future.cancel()` so that our callback can handle this case as well. For instance, if the Service Server has been shutdown, the :code:`Future` would never be done.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 67-72
   :emphasize-lines: 1-4

The Future callback
^^^^^^^^^^^^^^^^^^^

The callback for the :code:`Future` must receive a :code:`Future` as an argument. Having it as an attribute of the Node's class allows us to access ROS2 method such as :code:`get_logger()` and other contextual information.

The result of the :code:`Future` is obtained using :code:`Future.result()`. The response might be :code:`None` in some cases, so we must check it before trying to use the result, otherwise we will get a nasty exception.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/what_is_the_point_service_client_node.py
   :language: python
   :lines: 74-88
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

In another terminal, we run the :program:`what_is_the_point_service_server_node`, as follows

.. code:: console

    ros2 run python_package_that_uses_the_services what_is_the_point_service_server_node

The server Node will then output, periodically,

.. code:: console

   [INFO] [1684485151.608507798] [what_is_the_point_service_server]: 
   This is the call number 1 to this Service Server.
   The analysis of the AmazingQuote below is complete.

           [...] your living... it is always potatoes. I dream of potatoes.

   -- a young Maltese potato farmer

   The point has been sent back to the client.

   [INFO] [1684485152.092508332] [what_is_the_point_service_server]: 
   This is the call number 2 to this Service Server.
   The analysis of the AmazingQuote below is complete.

           I wonder about the Ultimate Question of Life, the Universe, and Everything.

   -- Creators of Deep Thought

   The point has been sent back to the client.

   [INFO] [1684485152.592516148] [what_is_the_point_service_server]: 
   This is the call number 3 to this Service Server.
   The analysis of the AmazingQuote below is complete.

           I wonder about the Ultimate Question of Life, the Universe, and Everything.

   -- Creators of Deep Thought

   The point has been sent back to the client.

and the client Node will output, periodically,

.. code:: console

   [INFO] [1684485151.609611689] [what_is_the_point_service_client]: 
   We have thus received the point of our quote.

               (18.199457100225292, 33.14595477433704, 52.65262570058381)

   [INFO] [1684485152.093228181] [what_is_the_point_service_client]: 
   We have thus received the point of our quote.

               (11.17170193214362, 9.384897014549527, 21.443401053306854)

   [INFO] [1684485152.593294259] [what_is_the_point_service_client]: 
   We have thus received the point of our quote.

               (16.58535176162403, 0.6180505400411676, 24.796597698334804)


