.. include:: the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

Call for Actions: Action Servers and Clients
============================================

What about a mixture of messages and services? That is where actions come into play.

We use actions by creating an :code:`ActionServer`. The :code:`ActionServer` called by one or more :code:`ActionClient`\s.

Similarly to a service, each action should only have a single :code:`ActionServer` that will receive a :code:`Goal` and provide a :code:`Result`.
It will also provide :code:`Feedback` through a suitable topic. It can be argued that the main difference between a service
and an action is the capability of providing feedback while the action is performed. A service, in contrast, only outputs
a single, final result of the service call.

Objective
---------

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