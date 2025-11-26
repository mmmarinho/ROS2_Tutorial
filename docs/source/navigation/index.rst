About Navigation
================

.. versionadded:: Jazzy

   This section.

.. include:: ../the_topic_is_under_heavy_construction.rst

.. seealso::

    Official documentation: https://docs.nav2.org

The concept of navigation is part of a wide range of robotics applications. It can usually be simply put as moving a
robot from one pose to another. Although in our human experience such task can be trivially understood, formally
transforming that into mathematical descriptions and their suitable implementation is far from trivial.

This section will cover a few navigation concepts, but it is not meant to be complete. Our interest is to use the
existing navigation packages in :program:`ROS2`, namely ``nav2``, through custom-made nodes. The tools we have seen
so far in :program:`ROS2` will allow us to understand and interact with ``nav2`` packages.

Nonetheless, there are sizable tutorials dedicated exclusively to navigation. This will not be the case here. The
focus will be on linking the concepts learned so far and showcasing how they can be combined to interact with existing
packages from the community. Conceptually, interacting with existing packages should be easy. However, going beyond
using existing examples will require, possibly, all the knowledge acquired in previous sections.

For illustrative purposes, we will also use the ``slam_toolbox``, meant for simultaneous localisation and mapping (SLAM).
Whenever a concept from SLAM coincides with a navigation topic, this will be highlighted. However, going into detail
about SLAM is out of the scope of this tutorial.
