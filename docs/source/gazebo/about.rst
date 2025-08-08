.. include:: ../the_topic_is_under_heavy_construction.rst

About :program:`Gazebo`
=======================

.. note::

   Click here for the `official docs <https://gazebosim.org/docs/harmonic/getstarted/>`_ from the developers of :program:`Gazebo`.

:program:`Gazebo` is a robot simulator frequently mentioned when :program:`ROS2` is used. They are officially different projects although there are `clear connections <https://gazebosim.org/docs/latest/ros_installation/#summary-of-compatible-ros-and-gazebo-combinations>`_.

Until :program:`ROS2 Humble`, :program:`Gazebo` worked in a different way and had a different name. The older project has been renamed to :program:`Gazebo Classic`.

Given that this new way to use :program:`Gazebo` has started in :program:`ROS2 Jazzy`, we can expect that some edges will be rough and some functionalities missing. Nonetheless, it is reasonable to believe this will be the way to operate :program:`Gazebo` in the foreseable future.

For :program:`ROS2 Jazzy`, we use :program:`Gazebo Harmonic`. This replicates the strategy of having a LTS version for a given piece of software. This makes it easier for the users to learn and trust a platform without it constantly changing.

Beauty and usability are in the eyes of the beholder. That said, one major improvement in :program:`Gazebo` is the motion towards not needing two robot description formats. It has always been the case that a robot would have to be described twice, once for :program:`ROS2` and another for :program:`Gazebo Classic`.
Although this switch might still be ongoing and some more advanced functionalities might be missing, we will work with a single ``.sdf`` file as much as possible in these tutorials.
