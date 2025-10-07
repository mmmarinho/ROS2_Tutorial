.. include:: ../the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

Call for Actions: Servers and Clients
=====================================

What about a mixture of :code:`Messages` and :code:`Services`? That is where :code:`Actions` come into play.

We use :code:`Actions` by creating an :code:`ActionServer`. The :code:`ActionServer` will provide an action that can be accessed by one or more :code:`ActionClient`\s.

Similarly to a :code:`Service`, each :code:`Action` should only have a single :code:`ActionServer` that will receive a :code:`Goal` and provide a :code:`Result`.
The :code:`Feedback` topic is XYZ.

Create the package
------------------

We start by creating a package to use the :code:`Service` we first created in :ref:`The service file`.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_the_actions \
    --build-type ament_python \
    --dependencies rclpy package_with_interfaces

.. dropdown:: ros2 pkg create output

   .. code :: console

        going to create a new package
        package name: python_package_that_uses_the_actions
        destination directory: ~/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['rclpy', 'package_with_interfaces']
        creating folder ./python_package_that_uses_the_actions
        creating ./python_package_that_uses_the_actions/package.xml
        creating source folder
        creating folder ./python_package_that_uses_the_actions/python_package_that_uses_the_actions
        creating ./python_package_that_uses_the_actions/setup.py
        creating ./python_package_that_uses_the_actions/setup.cfg
        creating folder ./python_package_that_uses_the_actions/resource
        creating ./python_package_that_uses_the_actions/resource/python_package_that_uses_the_actions
        creating ./python_package_that_uses_the_actions/python_package_that_uses_the_actions/__init__.py
        creating folder ./python_package_that_uses_the_actions/test
        creating ./python_package_that_uses_the_actions/test/test_copyright.py
        creating ./python_package_that_uses_the_actions/test/test_flake8.py
        creating ./python_package_that_uses_the_actions/test/test_pep257.py

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

    This will be the file structure for the :code:`Action` tutorial. Highlighted are the main files for the :code:`ActionServer` and :code:`ActionClient`.

    .. code-block:: console

        TODO

Before we start exploring the elements of the package, let us

#. Create the Node with an :code:`ActionServer`.
#. Create the Node with an :code:`ActionClient`.
#. Update the :file:`setup.py` so that :program:`ros2 run` finds these programs.

Action Server
-------------

We can use an Action to represent the first robot-like behavior in an illustrative manner. Suppose that we have a simple
robot that moves in 2D space and whose orientation is not important. Let it have a current position with respect to the
:math:`k`\-ith iteration represented by

.. math::

    \boldsymbol{p}(k)  =\begin{bmatrix}x(k) \\ y(k)\end{bmatrix}

and a desired position given by

.. math::

    \boldsymbol{p}_d  =\begin{bmatrix}x_d \\ y_d\end{bmatrix}.

Suppose that we want to design an action server that takes this robot-like object from its current position :math:`\boldsymbol{p}` and moves it
towards the goal :math:`\boldsymbol{p}_d` with a speed :math:`s \in \mathbb{R}`. As feedback, it gives us the distance :math:`d \in \mathbb{R}` between the current
position and the desired position.

While being conscious of our objectives, for any action server, it is important to:

#. Receive the goal and process it in a meaningful way.
#. Publish feedback as it becomes available. Without feedback, an Action might always be replaced more effectively by a Service.
#. Set the final state of the goal and send a Result.

Let us create the action server as follows.

:download:`move_straight_in_2d_action_server_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_server_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_server_node.py
   :language: python
   :lines: 24-
   :linenos:

As always, our class must inherit from :code:`rclpy.node.Node`, so that it can be used as argument of :code:`rclpy.spin()`.

An action server is an instance of :code:`rclpy.action.ActionServer`.
As input to the initializer of :code:`ActionServer`, we need an action, a string that names this action server, and a suitable callback.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_server_node.py
   :language: python
   :lines: 36-49
   :emphasize-lines: 1, 10

The callback is likely to be the most complex aspect of an action server. The callback must receive an instance of
:code:`ServerGoalHandle`. This will be managed automatically by ROS2. It must return an instance of the class :code:`MyAction.Result` instance for any action :code:`MyAction`
you are serving. This must be done manually by the programmer of the server. Note that the :code:`MyAction.Result` class is created for
you via :program:`colcon build`.

The request can be obtained using :code:`goal.request` of type :code:`MyAction.Request`. Given that our action has the field :code:`desired_position`
we can access it this way.

A feedback message must be sent as suitable. One can be created with the :code:`MyAction.Feedback` class, which is created for
you via :program:`colcon build`. After creating a suitable message, it can be sent with :code:`goal.publish_feedback`.

If the action is successful, it is important to set :code:`goal.succeed`. There are instances in which you would like to
abort the goal early. This does not apply to this example because it will fail naturally after it times out. If you do
not set a goal as succeeded, it will automatically be aborted at the end of the callback.

Lastly, the method must return an instance of :code:`MyAction.Result`. In our case, we have the field :code:`final_position`
in the action file description, therefore we populate it as needed.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_server_node.py
   :language: python
   :lines: 84-115
   :emphasize-lines: 1, 7, 12, 18, 24, 30, 32

The other methods are to support the important aspects of the action server. The :code:`get_distance` method will compute
the Euclidean distance between :code:`current_position` and :code:`desired_position`. The distance :math:`d` will be calculated
from the terms :math:`x`, :math:`x_d`, :math:`y`, and :math:`y_d`.

.. math::

    d = \sqrt{(x-x_d)^2 + (y-y_d)^2}

This is represented by the following piece of code.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_server_node.py
   :language: python
   :lines: 53-64
   :emphasize-lines: 1, 12

The action server will update the :code:`current_position` based on a simple constant speed motion of the point. This
can be mathematically described as follows.

.. math::

    x(k+1) = x(k) - s\left(\frac{x - x_d}{d}\right),

where in the :math:`k`\-ith iteration :math:`x(k+1)` represents the next position, :math:`x(k)` the current position, and :math:`s \in \mathbb{R}` is the desired speed.
This is represented by the following piece of code.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_server_node.py
   :language: python
   :lines: 66-81

Note that we use an arbitrary minimum distance as stop criteria. Do not expect the distance to ever be zero, given
that we are using floating point representation for the numbers. If you do not add a stopping range it is likely
that your node will be stuck forever.

Testing the Action Server
-------------------------

Run the action server with the following command.

.. code-block:: console

    ros2 run python_package_that_uses_the_actions move_straight_in_2d_action_server_node

In another terminal, run the following command to test the action server.

.. code-block:: console

    ros2 action send_goal --feedback \
    /move_straight_in_2d \
    package_with_interfaces/action/MoveStraightIn2D \
    '{
    desired_position:{
        x: 1.0,
        y: 0.0,
        z: 0.0}
    }'

This is what will be shown in each terminal

.. tab-set::

    .. tab-item:: ros2 run output

        .. code-block:: console

            [INFO] [1759841973.596577423] [move_straight_in_2d_action_server]: current_position is geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0).
            [INFO] [1759841973.596757548] [move_straight_in_2d_action_server]: desired_position set to geometry_msgs.msg.Point(x=1.0, y=0.0, z=0.0).

    .. tab-item:: ros2 action send_goal output

        .. code-block:: console

            Waiting for an action server to become available...
            Sending goal:
                 desired_position:
              x: 1.0
              y: 0.0
              z: 0.0

            Goal accepted with ID: 09c38042aaf846f49a87523dddf50900

            Feedback:
                distance: 1.0

            Feedback:
                distance: 0.9900000095367432

            Feedback:
                distance: 0.9800000190734863

            Feedback:
                distance: 0.9700000286102295

            Feedback:
                distance: 0.9599999785423279

            Feedback:
                distance: 0.949999988079071

            Feedback:
                distance: 0.9399999976158142

            Feedback:
                distance: 0.9300000071525574

            Feedback:
                distance: 0.9200000166893005

            Feedback:
                distance: 0.9100000262260437

            Feedback:
                distance: 0.8999999761581421

            Feedback:
                distance: 0.8899999856948853

            Feedback:
                distance: 0.8799999952316284

            Feedback:
                distance: 0.8700000047683716

            Feedback:
                distance: 0.8600000143051147

            Feedback:
                distance: 0.8500000238418579

            Feedback:
                distance: 0.8399999737739563

            Feedback:
                distance: 0.8299999833106995

            Feedback:
                distance: 0.8199999928474426

            Feedback:
                distance: 0.8100000023841858

            Feedback:
                distance: 0.800000011920929

            Feedback:
                distance: 0.7900000214576721

            Feedback:
                distance: 0.7799999713897705

            Feedback:
                distance: 0.7699999809265137

            Feedback:
                distance: 0.7599999904632568

            Feedback:
                distance: 0.75

            Feedback:
                distance: 0.7400000095367432

            Feedback:
                distance: 0.7300000190734863

            Feedback:
                distance: 0.7200000286102295

            Feedback:
                distance: 0.7099999785423279

            Feedback:
                distance: 0.699999988079071

            Feedback:
                distance: 0.6899999976158142

            Feedback:
                distance: 0.6800000071525574

            Feedback:
                distance: 0.6700000166893005

            Feedback:
                distance: 0.6600000262260437

            Feedback:
                distance: 0.6499999761581421

            Feedback:
                distance: 0.6399999856948853

            Feedback:
                distance: 0.6299999952316284

            Feedback:
                distance: 0.6200000047683716

            Feedback:
                distance: 0.6100000143051147

            Feedback:
                distance: 0.6000000238418579

            Feedback:
                distance: 0.5899999737739563

            Feedback:
                distance: 0.5799999833106995

            Feedback:
                distance: 0.5699999928474426

            Feedback:
                distance: 0.5600000023841858

            Feedback:
                distance: 0.550000011920929

            Feedback:
                distance: 0.5400000214576721

            Feedback:
                distance: 0.5299999713897705

            Feedback:
                distance: 0.5199999809265137

            Feedback:
                distance: 0.5099999904632568

            Feedback:
                distance: 0.5

            Feedback:
                distance: 0.49000000953674316

            Feedback:
                distance: 0.47999998927116394

            Feedback:
                distance: 0.4699999988079071

            Feedback:
                distance: 0.46000000834465027

            Feedback:
                distance: 0.44999998807907104

            Feedback:
                distance: 0.4399999976158142

            Feedback:
                distance: 0.4300000071525574

            Feedback:
                distance: 0.41999998688697815

            Feedback:
                distance: 0.4099999964237213

            Feedback:
                distance: 0.4000000059604645

            Feedback:
                distance: 0.38999998569488525

            Feedback:
                distance: 0.3799999952316284

            Feedback:
                distance: 0.3700000047683716

            Feedback:
                distance: 0.36000001430511475

            Feedback:
                distance: 0.3499999940395355

            Feedback:
                distance: 0.3400000035762787

            Feedback:
                distance: 0.33000001311302185

            Feedback:
                distance: 0.3199999928474426

            Feedback:
                distance: 0.3100000023841858

            Feedback:
                distance: 0.30000001192092896

            Feedback:
                distance: 0.28999999165534973

            Feedback:
                distance: 0.2800000011920929

            Feedback:
                distance: 0.27000001072883606

            Feedback:
                distance: 0.25999999046325684

            Feedback:
                distance: 0.25

            Feedback:
                distance: 0.23999999463558197

            Feedback:
                distance: 0.23000000417232513

            Feedback:
                distance: 0.2199999988079071

            Feedback:
                distance: 0.20999999344348907

            Feedback:
                distance: 0.20000000298023224

            Feedback:
                distance: 0.1899999976158142

            Feedback:
                distance: 0.18000000715255737

            Feedback:
                distance: 0.17000000178813934

            Feedback:
                distance: 0.1599999964237213

            Feedback:
                distance: 0.15000000596046448

            Feedback:
                distance: 0.14000000059604645

            Feedback:
                distance: 0.12999999523162842

            Feedback:
                distance: 0.11999999731779099

            Feedback:
                distance: 0.10999999940395355

            Feedback:
                distance: 0.10000000149011612

            Feedback:
                distance: 0.09000000357627869

            Feedback:
                distance: 0.07999999821186066

            Feedback:
                distance: 0.07000000029802322

            Feedback:
                distance: 0.05999999865889549

            Feedback:
                distance: 0.05000000074505806

            Feedback:
                distance: 0.03999999910593033

            Feedback:
                distance: 0.029999999329447746

            Feedback:
                distance: 0.019999999552965164

            Feedback:
                distance: 0.009999999776482582

            Result:
                final_position:
              x: 0.9900000000000007
              y: 0.0
              z: 0.0

            Goal finished with status: SUCCEEDED


You can try with different positions. As long as the action server node is not closed, it will keep the state.
As soon as the action server is closed, it will lose any internal state.

If the action server is unable to reach the desired goal within the time allowed in the action server, the goal
will finish with status :code:`ABORTED`. For instance, the last lines for a goal that is too far from the current position
would be as follows.

.. code-block:: console

    [...]

    Feedback:
        distance: 98.0199966430664

    Result:
        final_position:
      x: 1.9900000000000015
      y: 0.0
      z: 0.0

    Goal finished with status: ABORTED