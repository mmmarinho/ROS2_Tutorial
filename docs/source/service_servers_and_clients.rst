.. versionchanged:: Jazzy

   The contents of this session were expanded and the example simplified. The previous version is available at the `Humble tutorials <https://ros2-tutorial.readthedocs.io/en/humble/service_servers_and_clients.html>`_.

At your Service: Servers and Clients
====================================

In some cases, we need means of communication in which each command has an associated response. That is where :code:`Services` come into play. We use :code:`Services` by creating a :code:`ServiceServer`.
The :code:`ServiceServer` will provide a service that can be accessed by one or more :code:`ServiceClient`\s.

In this sense, a :code:`Service` is much less of an abstract entity than a :code:`Topic`.
Each :code:`Service` should only have a single :code:`ServiceServer` that will receive a :code:`Request` and provide a :code:`Response`.

.. mermaid:: Action client and server sequence diagram.

    %%{init: { "theme" : "dark" }}%%
    graph LR;
    A[Service Client #1] --> B[Service Server]
    C[Service Client #2] --> B
    B --> A
    B --> C

Create the package
------------------

We start by creating a package to use the :code:`Service` we first created in :ref:`The service file`.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_the_services \
    --build-type ament_python \
    --dependencies rclpy package_with_interfaces

.. dropdown:: ros2 pkg create output

   .. code :: console

        going to create a new package
        package name: python_package_that_uses_the_services
        destination directory: /root/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['rclpy', 'package_with_interfaces']
        creating folder ./python_package_that_uses_the_services
        creating ./python_package_that_uses_the_services/package.xml
        creating source folder
        creating folder ./python_package_that_uses_the_services/python_package_that_uses_the_services
        creating ./python_package_that_uses_the_services/setup.py
        creating ./python_package_that_uses_the_services/setup.cfg
        creating folder ./python_package_that_uses_the_services/resource
        creating ./python_package_that_uses_the_services/resource/python_package_that_uses_the_services
        creating ./python_package_that_uses_the_services/python_package_that_uses_the_services/__init__.py
        creating folder ./python_package_that_uses_the_services/test
        creating ./python_package_that_uses_the_services/test/test_copyright.py
        creating ./python_package_that_uses_the_services/test/test_flake8.py
        creating ./python_package_that_uses_the_services/test/test_pep257.py

        [WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
        It is recommended to use one of the ament license identifiers:
        Apache-2.0
        BSL-1.0
        BSD-2.0
        BSD-2-Clause
        BSD-3-Clause
        GPL-3.0-only
        LGPL-3.0-only
        MIT
        MIT-0

Overview
--------

.. admonition:: File structure

    This will be the file structure for the :code:`Service` tutorial. Highlighted are the main files for the :code:`ServiceServer` and :code:`ServiceClient`.

    .. code-block:: console
        :emphasize-lines: 6,8

        python_package_that_uses_the_services
        |-- package.xml
        |-- python_package_that_uses_the_services
        |   |-- __init__.py
        |   |-- add_points_service_client_introspection_node.py
        |   |-- add_points_service_client_node.py
        |   |-- add_points_service_server_introspection_node.py
        |   `-- add_points_service_server_node.py
        |-- resource
        |   `-- python_package_that_uses_the_services
        |-- setup.cfg
        |-- setup.py
        `-- test
            |-- test_copyright.py
            |-- test_flake8.py
            `-- test_pep257.py

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

Let's start by creating a :file:`add_points_service_server_node.py`.

:download:`~/ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_server_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_server_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_server_node.py
   :language: python
   :lines: 24-

The code begins with an import to the service we created. No surprise here.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_server_node.py
   :language: python
   :lines: 24-29
   :emphasize-lines: 3

The Service Server must be initialised with the :code:`create_service()`, as follows, with parameters that should by now be quite obvious to us.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_server_node.py
   :language: python
   :lines: 35-38

The Service Server receives a :code:`AddPoints.Request` and returns a :code:`AddPoints.Response`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_server_node.py
   :language: python
   :lines: 42-48
   :emphasize-lines: 2,4

.. warning::

   The API for the Service Server callback is a bit weird in that needs the :code:`Response` as an argument.
   This API `might change <https://github.com/ros2/rclpy/issues/464>`_, but for now this is what we got.

We use the members of :code:`AddPoints.Request` to calculate and populate the :code:`AddPoints.Response`. At the end of the callback, we must return that :code:`AddPoints.Request`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_server_node.py
   :language: python
   :lines: 50-54

The Service Server was quite painless, but it doesn't do much. The Service Client might be a bit more on the painful side for the uninitiated.

Service Clients
---------------

In :code:`rclpy`, service clients are implemented using an :code:`asyncio` logic (`more info <https://docs.python.org/3.12/library/asyncio.html>`_).
In this tutorial, we briefly introduce unavoidable :code:`async` concepts in :ref:`Asyncio`. For extra understanding, please check the official documentation.

The reasons for :code:`asyncio` are very simple. We do not know when the :code:`Response` of a service will be available. Then, when a service is called, we do not want our :code:`Node` with a :code:`ServiceClient` to get stuck while waiting for it.
Using :code:`asyncio` allows the :code:`Node` to do other things while the :code:`Response` is received and processed.

In contrast with a :code:`Message`, we need to worry about blocking the :code:`Node` with a :code:`ServiceServer` because a :code:`Service` demands a response.

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

We start by adding a :file:`add_points_service_client_node.py` at :file:`python_package_that_uses_the_services/python_package_that_uses_the_services` with the following contents.

:download:`add_points_service_client_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :linenos:
   :lines: 24-

Imports
^^^^^^^

To have access to the service, we import it with :code:`from <package>.srv import <Service>`. 

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 24-30
   :emphasize-lines: 5,7

Instantiate a Service Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We instantiate a Service Client with :code:`Node.create_client()`. The values of :code:`srv_type` and :code:`srv_name` must match the ones used in the Service Server.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 39-41

(Recommended) Wait for the Service Server to be available
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
   The order of execution and speed of Nodes depend on a complicated web of relationships between ROS2, the operating system, and the workload of the machine. It would be naive to expect the server to always be active before the client, even if the server Node is started before the client Node.

In many cases, having the result of the service is of particular importance (hence the use of a service and not messages). In that case, we have to wait until :code:`service_client.wait_for_service()`, as shown below.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 43,44

Instantiate a :code:`Future` as a class attribute
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As part of the :code:`async` framework, we instantiate a :code:`Future` (`More info <https://docs.python.org/3.10/library/asyncio-future.html#asyncio-futures>`_). In this example it is important to have it as an attribute of the class so that we do not lose the reference to it after the callback.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 46

Instantiate a Timer
^^^^^^^^^^^^^^^^^^^

Whenever periodic work must be done, it is recommended to use a :code:`Timer`, as we already learned in :ref:`Use a Timer for periodic work`.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 48-51

The need for a callback for the :code:`Timer`, should also be no surprise.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 53-54

Service Clients use :code:`<srv>.Request()`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Given that services work in a request-response model, the Service Client must instantiate a suitable :code:`<srv>.Request()` and populate its fields before making the service call, as shown below. To make the example more interesting, it randomly switches between two possible quotes.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 56-64

Make service calls with :code:`call_async()`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :code:`async` framework in ROS2 is based on Python's :code:`asyncio` that we already saw in :ref:`Asyncio`.
   
.. note::
   At first glance, it might feel that all this trouble to use :code:`async` is unjustified. However, Nodes in practice will hardly ever do one service call and be done. Many Nodes in a complex system
   will have a composition of many service servers, service clients, publishers, and subscribers. Blocking the entire Node while it waits for the result of a service is, in most cases, a bad design.

The recommended way to call a service is through :code:`call_async()`, which is the reason why we are working with :code:`async` logic. In general, the result of :code:`call_async()`, a :code:`Future`, will not have the result of the service call at the next line of our program. 

There are many ways to address the use of a :code:`Future`. One of them, specially tailored to interface :code:`async` with callback-based frameworks is the :code:`Future.add_done_callback()`. If the :code:`Future` is already done by the time we call :code:`add_done_callback()`, it is supposed to `call the callback for us <https://github.com/ros2/rclpy/blob/0f1af0db16c38899aaea1fb1ca696800255d2b55/rclpy/rclpy/task.py#L163>`_.

The benefit of this is that the callback will not block our resources until the response is ready. When the response is ready, and the ROS2 executor gets to processing :code:`Future` callbacks, our callback will be called *automagically*.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 66-71
   :emphasize-lines: 5,6

Given that we are periodically calling the service, before replace the class :code:`Future` with the next service call, we can check if the service call was done with :code:`Future.done()`. If it is not done, we can use :code:`Future.cancel()` so that our callback can handle this case as well. For instance, if the Service Server has been shutdown, the :code:`Future` would never be done.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 66-71
   :emphasize-lines: 1-4

The Future callback
^^^^^^^^^^^^^^^^^^^

The callback for the :code:`Future` must receive a :code:`Future` as an argument. Having it as an attribute of the Node's class allows us to access ROS2 method such as :code:`get_logger()` and other contextual information.

The result of the :code:`Future` is obtained using :code:`Future.result()`. The response might be :code:`None` in some cases, so we must check it before trying to use the result, otherwise we will get a nasty exception.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_services/python_package_that_uses_the_services/add_points_service_client_node.py
   :language: python
   :lines: 73-79
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

   ros2 run python_package_that_uses_the_services add_points_service_client_node

when running the client Node, the server is still not active. In that case, the client node will keep waiting for it, as follows

.. code:: console

    [INFO] [1753667386.959416097] [add_points_service_client]: service /add_points not available, waiting...
    [INFO] [1753667387.967904375] [add_points_service_client]: service /add_points not available, waiting...
    [INFO] [1753667388.978200250] [add_points_service_client]: service /add_points not available, waiting...

In another terminal, we run the :program:`add_points_service_server_node`, as follows

.. code:: console

    ros2 run python_package_that_uses_the_services add_points_service_server_node

The server Node will output nothing, whereas the client Node will output, periodically,

.. code:: console

    [INFO] [1753667415.876223138] [add_points_service_client]: The result was (853.122385593111, 613.3399959983066, 722.6376752208978)
    [INFO] [1753667416.373657638] [add_points_service_client]: The result was (645.418992882397, 560.9466217293334, 874.7214190239486)
    [INFO] [1753667416.875945305] [add_points_service_client]: The result was (1270.7448356640075, 345.69676803639936, 953.6879012399689)
    [INFO] [1753667417.376013639] [add_points_service_client]: The result was (1203.944887411107, 733.5131783020975, 927.902266740569)
    [INFO] [1753667417.872921291] [add_points_service_client]: The result was (671.9458297091917, 1210.490009902154, 545.6078440547075)
    [INFO] [1753667418.371451541] [add_points_service_client]: The result was (629.0766519110047, 872.0699525880541, 581.7396957576223)
    [INFO] [1753667418.873213958] [add_points_service_client]: The result was (579.0485532702639, 1714.0365146695003, 396.1743388215037)
    [INFO] [1753667419.375147834] [add_points_service_client]: The result was (545.2849740451343, 1629.1832720438556, 945.1871456532875)


