.. include:: the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

Action Clients
==============

An action client will be much like a service client, but more complicated. For instance, they will use :code:`async` and :code:`Future`\s.
The reason for that is that processing the :code:`Action` requires multiple steps.

To simplify the action server, our example code will only call the :code:`Action` once and do nothing else. Remember
that when deploying actions in real applications they will be part of a more complex :code:`Node` that might include
publishers, subscribers, service servers/clients, and other actions server/clients. Therefore it is important to take
this complexity in consideration when designing your packages to make sure that an :code:`Action` is the best way to communicate.

Diagram
-------

This is the sequence diagram from the point of view of the action client.

.. mermaid::

    ---
    config:
      theme: redux-dark-color
      look: neo
    ---
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

Overview
--------

#. Create the Node with an :code:`ActionClient`.
#. Update the :file:`setup.py` so that :program:`ros2 run` finds the node (if needed).

:download:`move_straight_in_2d_action_client_node.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_actions/python_package_that_uses_the_actions/move_straight_in_2d_action_client_node.py
   :language: python
   :lines: 24-
   :linenos:

