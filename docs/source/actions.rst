Call for Actions: Action Servers and Clients
============================================

.. include:: the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

What about a mixture of messages and services? That is where actions come into play.

We use actions by creating an :code:`ActionServer`. The :code:`ActionServer` can be called by one or more :code:`ActionClient`\s.

Similarly to a service, each action should only have a single :code:`ActionServer` that will receive a goal (with a :code:`Request`) and provide
a :code:`Result`. It will also provide :code:`Feedback` through a suitable topic.

It can be argued that the main difference
between a service and an action is the capability of providing feedback while the action is performed. A service, in contrast,
only outputs a single, final result of the service call.

Objective
---------

.. note::

    Unless otherwise explicitly stated, we will always follow the `International System of Units <https://en.wikipedia.org/wiki/International_System_of_Units>`_.
    This means distances in meters, angles in radians, time in seconds, and so on.

We can use an action to represent the first robot-like behavior of this tutorial in an illustrative manner.

Suppose that we have a simple robot that moves in 2D space and whose orientation is not important. Let it have a current position with respect to the
:math:`k`\-ith iteration represented by (a 2D vector)

.. math::

    \mathbb{R}^2 \ni \boldsymbol{p}(k)  =\begin{bmatrix}x(k) \\ y(k)\end{bmatrix}

and a desired position given by

.. math::

    \mathbb{R}^2 \ni \boldsymbol{p}_d  =\begin{bmatrix}x_d \\ y_d\end{bmatrix}.

Suppose that we want to design an action server that takes this robot-like object from its current position :math:`\boldsymbol{p}` and moves it
towards the goal :math:`\boldsymbol{p}_d` with a speed :math:`s \in \mathbb{R}` (in [m/s]). As feedback, it gives us the distance :math:`d \in \mathbb{R}` between the current
position and the desired position.

The (Euclidean) distance :math:`d` will be calculated
from the terms :math:`x`, :math:`x_d`, :math:`y`, and :math:`y_d`, as follows.

.. math::
    :name: eq:actions_distance

    d = \sqrt{(x-x_d)^2 + (y-y_d)^2}.

The action server will update the position :math:`\boldsymbol{p}(k)` based on a simple constant speed motion of the robot.
This can be mathematically described as follows.

.. math::
    :name: eq:actions_controller

    \boldsymbol{p}(k+1) = \boldsymbol{p}(k) - s\left(\frac{\boldsymbol{p} - \boldsymbol{p}_d}{d}\right)T,

where :math:`\boldsymbol{p}(k+1)` represents the next position, :math:`\boldsymbol{p}(k)` the current position,
and :math:`T` is the sampling time.

Create the package
------------------

We start by creating a package to use the action we first created in :ref:`The action file`.

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

This will be the file structure for the :code:`Action` tutorial. Highlighted are the main files for the :code:`ActionServer` and :code:`ActionClient`.

.. admonition:: File structure

    .. code-block:: console
        :emphasize-lines: 5,6

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