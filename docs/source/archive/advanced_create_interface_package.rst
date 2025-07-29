Creating a dedicated package for custom interfaces (with :program:`ament_auto`)
===============================================================================

.. warning::

   The :program:`ament_auto` directives can come in handy, but are poorly documented and poorly supported. Use at your own risk.

.. note::
 
   In ROS2 Humble, :file:`tf2_sensor_msgs` was the only package in the core libraries using :program:`ament_auto` and even that one was replaced by the common directives in ROS2 Rolling
   
   - Humble: https://github.com/ros2/geometry2/blob/d508d9c312e8cfa69e81bf5e53af38849f2bc870/tf2_sensor_msgs/package.xml
   - Rolling: https://github.com/ros2/geometry2/blob/ac2479e181a9d0a3efbc630ef55dfa5dd7afb1a9/tf2_sensor_msgs/package.xml

Refs

- https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html
- https://gist.github.com/dirk-thomas/a76f952d05e7b21b0128
- https://github.com/ament/ament_cmake/issues/301
- https://design.ros2.org/articles/ament.html
