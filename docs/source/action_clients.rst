Action Clients
==============

.. include:: the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

An action client will be much like a service client, but more complicated. There are at least three callbacks and
two futures involved.

The reason for that is that processing the :code:`Action` requires multiple steps.

To simplify the action server, our example code will only call the :code:`Action` once and do nothing else.

Remember that when deploying actions in real applications they will be part of a more complex :code:`Node` that might include
publishers, subscribers, service servers/clients, and other actions server/clients. This means that it is important to take
this complexity in consideration when designing your packages to make sure that an :code:`Action` is the best way to communicate.

Diagram
-------

This is the sequence diagram from the point of view of the action client. Note that because we are using :code:`async`
calls for the goal and the result, the node is free to do other tasks while those do not arrive.

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    sequenceDiagram
      participant Action Client as Action Client
      participant Action Server as Action Server
      autonumber
      Action Client ->>+ Action Server: action_client.send_goal_async()
      Action Server -->>- Action Client: ActionClientNode.goal_response_callback()
      Action Client ->>+ Action Server: action_client.get_result_async()
      loop While action has not ended
        Action Server -->> Action Client: ActionClientNode.action_feedback_callback()
      end
      Action Server -->>- Action Client: ActionClientNode.action_result_callback()

Files
-----

The highlighted file below will be modified or created in this section.

.. admonition:: File structure

    .. code-block:: console
        :emphasize-lines: 5

        python_package_that_uses_the_actions/
        |-- package.xml
        |-- python_package_that_uses_the_actions
        |   |-- __init__.py
        |   |-- move_straight_in_2d_action_client_node.py
        |   `-- move_straight_in_2d_action_server_node.py
        |-- resource
        |   `-- python_package_that_uses_the_actions
        |-- setup.cfg
        |-- setup.py
        `-- test
            |-- test_copyright.py
            |-- test_flake8.py
            `-- test_pep257.py

Action Client
-------------

#. Create the Node with an :code:`ActionClient`.
#. Update the :file:`setup.py` so that :program:`ros2 run` finds the node (if needed).

:download:`move_straight_in_2d_action_client_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py
   :language: python
   :lines: 24-
   :linenos:

As always, our class must inherit from :code:`rclpy.node.Node`, so that it can be used as argument of :code:`rclpy.spin()`.

An action client is an instance of :code:`rclpy.action.ActionClient`.
As input to the initializer of :code:`ActionClient`, we need an action and a string that names the action server this client
will interact with.

In the action client, we will have to manage two instances of :code:`Future`. One for the goal and another for the result.
It is important to keep them as attributes
of the instance to guarantee they will not get out of scope before the result is obtained. This is done as follows.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py
   :language: python
   :lines: 36-42
   :emphasize-lines: 4,6,7

The following method can be used to send the action goal and trigger the entire process. Notably, suppose that a suitable
action has already been instantiated and is sent as argument to this method. We wait for the action server to be available.
Then, in something that should be familiar after working with services, we send the request asynchronously.

In the request, :code:`send_goal_async`, we define a callback for the feedback. This is similar to a callback for subscribers.
Whenever a feedback is sent, the method we assign as callback will be called.

After we send the goal request and assign a callback for the feedback, we assign another callback. This second callback
is for the result of the goal request operation.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py
   :language: python
   :lines: 44-54
   :emphasize-lines: 1,5,10,11

After the goal is processed, the goal-related callback will be automatically called for us given that it was added to the respective
future. The goal can be rejected, therefore we address that case by doing nothing. If the goal is accepted, we are ready
to ask for a result.

We ask for a result asynchronously. We then assign a third callback for when a response is sent.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py
   :language: python
   :lines: 56-65
   :emphasize-lines: 1,4,9,10

The result callback will receive a future and process the result of the action somehow. In this toy example, we simply
print the results to the screen.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py
   :language: python
   :lines: 67-69

Lastly, we see that the callback for the feedback is, again, very similar to that for a subscriber.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py
   :language: python
   :lines: 71-73

To simplify this example slightly, please notice that the action is instantiated in the :code:`main` function, and
the action is triggered only once.

When doing so, please remember to call the action after the node is instantiated and before :code:`rclpy.spin()`. Otherwise,
the object will not yet exist, or you will be locked in the spinner before sending the goal.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py
   :language: python
   :lines: 84-92

Build and source
----------------

Before we proceed, let us build and source once.

.. include:: the_canonical_build_command.rst

Testing the Action Client
-------------------------

We will be working with two terminal windows. The first will run the action server. The second, the client.

.. tab-set::

    .. tab-item:: Terminal 1: run action server

        .. code-block:: console

            ros2 run python_package_that_uses_the_actions move_straight_in_2d_action_server_node

    .. tab-item:: Terminal 2: run action client

        .. code-block:: console

            ros2 run python_package_that_uses_the_actions move_straight_in_2d_action_client_node

The output in each window should be something similar to

.. tab-set::

    .. tab-item:: Terminal 1: action server output

        .. code-block:: console

            [INFO] [1761753120.100902419] [move_straight_in_2d_action_server]: current_position is geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0).
            [INFO] [1761753120.101090669] [move_straight_in_2d_action_server]: desired_position set to geometry_msgs.msg.Point(x=1.0, y=-1.0, z=0.0).

    .. tab-item:: Terminal 2: action output

        .. code-block:: console

            [INFO] [1761753120.092259753] [move_straight_in_2d_action_client]: Sending goal: geometry_msgs.msg.Point(x=1.0, y=-1.0, z=0.0).
            [INFO] [1761753120.093185086] [move_straight_in_2d_action_client]: Goal was accepted by the server.
            [INFO] [1761753120.101412211] [move_straight_in_2d_action_client]: Received feedback distance: 1.4142135381698608.
            [INFO] [1761753120.112173919] [move_straight_in_2d_action_client]: Received feedback distance: 1.404213547706604.
            [INFO] [1761753120.123090294] [move_straight_in_2d_action_client]: Received feedback distance: 1.3942135572433472.
            [INFO] [1761753120.134245086] [move_straight_in_2d_action_client]: Received feedback distance: 1.3842135667800903.
            [INFO] [1761753120.145173920] [move_straight_in_2d_action_client]: Received feedback distance: 1.3742135763168335.
            [INFO] [1761753120.156160961] [move_straight_in_2d_action_client]: Received feedback distance: 1.3642135858535767.
            [INFO] [1761753120.167298045] [move_straight_in_2d_action_client]: Received feedback distance: 1.3542135953903198.
            [INFO] [1761753120.178755836] [move_straight_in_2d_action_client]: Received feedback distance: 1.344213604927063.
            [INFO] [1761753120.189400378] [move_straight_in_2d_action_client]: Received feedback distance: 1.3342136144638062.
            [INFO] [1761753120.200484253] [move_straight_in_2d_action_client]: Received feedback distance: 1.3242135047912598.
            [INFO] [1761753120.211245128] [move_straight_in_2d_action_client]: Received feedback distance: 1.314213514328003.
            [INFO] [1761753120.222122295] [move_straight_in_2d_action_client]: Received feedback distance: 1.304213523864746.
            [INFO] [1761753120.234860420] [move_straight_in_2d_action_client]: Received feedback distance: 1.2942135334014893.
            [INFO] [1761753120.245268795] [move_straight_in_2d_action_client]: Received feedback distance: 1.2842135429382324.
            [INFO] [1761753120.257182295] [move_straight_in_2d_action_client]: Received feedback distance: 1.2742135524749756.
            [INFO] [1761753120.268218170] [move_straight_in_2d_action_client]: Received feedback distance: 1.2642135620117188.
            [INFO] [1761753120.279651420] [move_straight_in_2d_action_client]: Received feedback distance: 1.254213571548462.
            [INFO] [1761753120.290934753] [move_straight_in_2d_action_client]: Received feedback distance: 1.244213581085205.
            [INFO] [1761753120.302183086] [move_straight_in_2d_action_client]: Received feedback distance: 1.2342135906219482.
            [INFO] [1761753120.313913295] [move_straight_in_2d_action_client]: Received feedback distance: 1.2242136001586914.
            [INFO] [1761753120.325966795] [move_straight_in_2d_action_client]: Received feedback distance: 1.2142136096954346.
            [INFO] [1761753120.336757795] [move_straight_in_2d_action_client]: Received feedback distance: 1.2042136192321777.
            [INFO] [1761753120.347904086] [move_straight_in_2d_action_client]: Received feedback distance: 1.1942135095596313.
            [INFO] [1761753120.359854920] [move_straight_in_2d_action_client]: Received feedback distance: 1.1842135190963745.
            [INFO] [1761753120.371660170] [move_straight_in_2d_action_client]: Received feedback distance: 1.1742135286331177.
            [INFO] [1761753120.382972545] [move_straight_in_2d_action_client]: Received feedback distance: 1.1642135381698608.
            [INFO] [1761753120.396173336] [move_straight_in_2d_action_client]: Received feedback distance: 1.154213547706604.
            [INFO] [1761753120.406693836] [move_straight_in_2d_action_client]: Received feedback distance: 1.1442135572433472.
            [INFO] [1761753120.418283586] [move_straight_in_2d_action_client]: Received feedback distance: 1.1342135667800903.
            [INFO] [1761753120.430012878] [move_straight_in_2d_action_client]: Received feedback distance: 1.1242135763168335.
            [INFO] [1761753120.440861920] [move_straight_in_2d_action_client]: Received feedback distance: 1.1142135858535767.
            [INFO] [1761753120.452906586] [move_straight_in_2d_action_client]: Received feedback distance: 1.1042135953903198.
            [INFO] [1761753120.464116961] [move_straight_in_2d_action_client]: Received feedback distance: 1.094213604927063.
            [INFO] [1761753120.475175961] [move_straight_in_2d_action_client]: Received feedback distance: 1.0842136144638062.
            [INFO] [1761753120.486437628] [move_straight_in_2d_action_client]: Received feedback distance: 1.0742135047912598.
            [INFO] [1761753120.497304670] [move_straight_in_2d_action_client]: Received feedback distance: 1.064213514328003.
            [INFO] [1761753120.508036878] [move_straight_in_2d_action_client]: Received feedback distance: 1.054213523864746.
            [INFO] [1761753120.519005753] [move_straight_in_2d_action_client]: Received feedback distance: 1.0442135334014893.
            [INFO] [1761753120.529962128] [move_straight_in_2d_action_client]: Received feedback distance: 1.0342135429382324.
            [INFO] [1761753120.540856920] [move_straight_in_2d_action_client]: Received feedback distance: 1.0242135524749756.
            [INFO] [1761753120.551795461] [move_straight_in_2d_action_client]: Received feedback distance: 1.0142135620117188.
            [INFO] [1761753120.563487586] [move_straight_in_2d_action_client]: Received feedback distance: 1.004213571548462.
            [INFO] [1761753120.575029170] [move_straight_in_2d_action_client]: Received feedback distance: 0.9942135810852051.
            [INFO] [1761753120.585385753] [move_straight_in_2d_action_client]: Received feedback distance: 0.9842135906219482.
            [INFO] [1761753120.596445128] [move_straight_in_2d_action_client]: Received feedback distance: 0.9742135405540466.
            [INFO] [1761753120.607320045] [move_straight_in_2d_action_client]: Received feedback distance: 0.9642135500907898.
            [INFO] [1761753120.617953295] [move_straight_in_2d_action_client]: Received feedback distance: 0.954213559627533.
            [INFO] [1761753120.628741545] [move_straight_in_2d_action_client]: Received feedback distance: 0.9442135691642761.
            [INFO] [1761753120.640129795] [move_straight_in_2d_action_client]: Received feedback distance: 0.9342135787010193.
            [INFO] [1761753120.651860920] [move_straight_in_2d_action_client]: Received feedback distance: 0.9242135882377625.
            [INFO] [1761753120.664198961] [move_straight_in_2d_action_client]: Received feedback distance: 0.9142135381698608.
            [INFO] [1761753120.674831961] [move_straight_in_2d_action_client]: Received feedback distance: 0.904213547706604.
            [INFO] [1761753120.686480336] [move_straight_in_2d_action_client]: Received feedback distance: 0.8942135572433472.
            [INFO] [1761753120.696874711] [move_straight_in_2d_action_client]: Received feedback distance: 0.8842135667800903.
            [INFO] [1761753120.708105420] [move_straight_in_2d_action_client]: Received feedback distance: 0.8742135763168335.
            [INFO] [1761753120.720106128] [move_straight_in_2d_action_client]: Received feedback distance: 0.8642135858535767.
            [INFO] [1761753120.731688795] [move_straight_in_2d_action_client]: Received feedback distance: 0.854213535785675.
            [INFO] [1761753120.744141753] [move_straight_in_2d_action_client]: Received feedback distance: 0.8442135453224182.
            [INFO] [1761753120.755073711] [move_straight_in_2d_action_client]: Received feedback distance: 0.8342135548591614.
            [INFO] [1761753120.767188003] [move_straight_in_2d_action_client]: Received feedback distance: 0.8242135643959045.
            [INFO] [1761753120.778392795] [move_straight_in_2d_action_client]: Received feedback distance: 0.8142135739326477.
            [INFO] [1761753120.789267920] [move_straight_in_2d_action_client]: Received feedback distance: 0.8042135834693909.
            [INFO] [1761753120.800152128] [move_straight_in_2d_action_client]: Received feedback distance: 0.7942135334014893.
            [INFO] [1761753120.811231461] [move_straight_in_2d_action_client]: Received feedback distance: 0.7842135429382324.
            [INFO] [1761753120.822597753] [move_straight_in_2d_action_client]: Received feedback distance: 0.7742135524749756.
            [INFO] [1761753120.833374087] [move_straight_in_2d_action_client]: Received feedback distance: 0.7642135620117188.
            [INFO] [1761753120.845155503] [move_straight_in_2d_action_client]: Received feedback distance: 0.7542135715484619.
            [INFO] [1761753120.857314462] [move_straight_in_2d_action_client]: Received feedback distance: 0.7442135810852051.
            [INFO] [1761753120.868129378] [move_straight_in_2d_action_client]: Received feedback distance: 0.7342135906219482.
            [INFO] [1761753120.879274587] [move_straight_in_2d_action_client]: Received feedback distance: 0.7242135405540466.
            [INFO] [1761753120.890035378] [move_straight_in_2d_action_client]: Received feedback distance: 0.7142135500907898.
            [INFO] [1761753120.900985587] [move_straight_in_2d_action_client]: Received feedback distance: 0.704213559627533.
            [INFO] [1761753120.911983378] [move_straight_in_2d_action_client]: Received feedback distance: 0.6942135691642761.
            [INFO] [1761753120.923409587] [move_straight_in_2d_action_client]: Received feedback distance: 0.6842135787010193.
            [INFO] [1761753120.934035170] [move_straight_in_2d_action_client]: Received feedback distance: 0.6742135882377625.
            [INFO] [1761753120.945181837] [move_straight_in_2d_action_client]: Received feedback distance: 0.6642135381698608.
            [INFO] [1761753120.957020503] [move_straight_in_2d_action_client]: Received feedback distance: 0.654213547706604.
            [INFO] [1761753120.968711462] [move_straight_in_2d_action_client]: Received feedback distance: 0.6442135572433472.
            [INFO] [1761753120.980621087] [move_straight_in_2d_action_client]: Received feedback distance: 0.6342135667800903.
            [INFO] [1761753120.991584587] [move_straight_in_2d_action_client]: Received feedback distance: 0.6242135763168335.
            [INFO] [1761753121.003069045] [move_straight_in_2d_action_client]: Received feedback distance: 0.6142135858535767.
            [INFO] [1761753121.015031753] [move_straight_in_2d_action_client]: Received feedback distance: 0.604213535785675.
            [INFO] [1761753121.027384962] [move_straight_in_2d_action_client]: Received feedback distance: 0.5942135453224182.
            [INFO] [1761753121.037966962] [move_straight_in_2d_action_client]: Received feedback distance: 0.5842135548591614.
            [INFO] [1761753121.048998128] [move_straight_in_2d_action_client]: Received feedback distance: 0.5742135643959045.
            [INFO] [1761753121.061886295] [move_straight_in_2d_action_client]: Received feedback distance: 0.5642135739326477.
            [INFO] [1761753121.072371337] [move_straight_in_2d_action_client]: Received feedback distance: 0.5542135834693909.
            [INFO] [1761753121.084468795] [move_straight_in_2d_action_client]: Received feedback distance: 0.5442135334014893.
            [INFO] [1761753121.096903712] [move_straight_in_2d_action_client]: Received feedback distance: 0.5342135429382324.
            [INFO] [1761753121.107056378] [move_straight_in_2d_action_client]: Received feedback distance: 0.5242135524749756.
            [INFO] [1761753121.118114628] [move_straight_in_2d_action_client]: Received feedback distance: 0.5142135620117188.
            [INFO] [1761753121.128596212] [move_straight_in_2d_action_client]: Received feedback distance: 0.5042135715484619.
            [INFO] [1761753121.140370212] [move_straight_in_2d_action_client]: Received feedback distance: 0.4942135512828827.
            [INFO] [1761753121.152598753] [move_straight_in_2d_action_client]: Received feedback distance: 0.48421356081962585.
            [INFO] [1761753121.165302628] [move_straight_in_2d_action_client]: Received feedback distance: 0.474213570356369.
            [INFO] [1761753121.178417045] [move_straight_in_2d_action_client]: Received feedback distance: 0.4642135500907898.
            [INFO] [1761753121.189332878] [move_straight_in_2d_action_client]: Received feedback distance: 0.45421355962753296.
            [INFO] [1761753121.199914878] [move_straight_in_2d_action_client]: Received feedback distance: 0.4442135691642761.
            [INFO] [1761753121.211306087] [move_straight_in_2d_action_client]: Received feedback distance: 0.4342135488986969.
            [INFO] [1761753121.223468503] [move_straight_in_2d_action_client]: Received feedback distance: 0.42421355843544006.
            [INFO] [1761753121.237240878] [move_straight_in_2d_action_client]: Final position was: geometry_msgs.msg.Point(x=0.7071067811865476, y=-0.7071067811865476, z=0.0).