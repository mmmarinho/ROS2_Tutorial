Messages and Services (:program:`ros2 interface`)
=================================================

If by now you haven't particularly fallen in love with ROS2, fear not. Indeed, we haven't done much so far that couldn't be achieved more easily by other means.

ROS2 begins to shine most in its interprocess communication, through what are called `ROS2 interfaces <https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html>`_. In particular, the fact that we can easily interface Nodes written in Python and C++ is a strong selling point.

:code:`Messages` are one of the three types of ROS2 interfaces. This will most likely be the standard of communication between Nodes in your packages. We will also see the bidirectional :code:`Services` now. The last type of interface, :code:`Actions`, is left for another section.

Description
-----------

In ROS2, interfaces are files written in the ROS2 :abbr:`IDL (Interface Description Language)`. Each type of interface is described in a :file:`.msg` file (or :file:`.srv` file), which is then built by :program:`colcon` into libraries that can be imported into your Python programs.

When dealing with common robotics concepts such as geometric and sensor messages, it is good practice to use interfaces that already exist in ROS2, instead of creating new ones that serve the exact same purpose. In addition, for complicated interfaces, we can combine existing ones for simplicity. 

Getting info on interfaces
--------------------------

We can get information about ROS2 interfaces available in our system with :program:`ros2 interface`. Let us first get more information about the program usage with

.. code:: console
   
   ros2 interface -h
   
which results in

.. code:: console

    usage: ros2 interface [-h] Call `ros2 interface <command> -h` for more detailed usage. ...

    Show information about ROS interfaces

    options:
      -h, --help            show this help message and exit

    Commands:
      list      List all interface types available
      package   Output a list of available interface types within one package
      packages  Output a list of packages that provide interfaces
      proto     Output an interface prototype
      show      Output the interface definition

      Call `ros2 interface <command> -h` for more detailed usage.
      
This shows that with :program:`ros2 interface list` we can get a list of all interfaces available in our workspace. That returns a huge list of interfaces, so it will not be replicated entirely here. Instead, we can run

.. code:: console

     ros2 interface packages
     
to get the list of packages with interfaces available, which returns something similar to
 
.. code-block:: console
    :emphasize-lines: 8, 19
 
    action_msgs
    action_tutorials_interfaces
    actionlib_msgs
    builtin_interfaces
    composition_interfaces
    diagnostic_msgs
    example_interfaces
    geometry_msgs
    lifecycle_msgs
    logging_demo
    map_msgs
    nav_msgs
    pcl_msgs
    pendulum_msgs
    rcl_interfaces
    rmw_dds_common
    rosbag2_interfaces
    rosgraph_msgs
    sensor_msgs
    shape_msgs
    statistics_msgs
    std_msgs
    std_srvs
    stereo_msgs
    tf2_msgs
    trajectory_msgs
    turtlesim
    unique_identifier_msgs
    visualization_msgs


From those, :file:`sensor_msgs` and :file:`geometry_msgs` are packages to always keep in mind when looking for a suitable interface. It will help to keep your Nodes compatible with the community.

.. warning:: 

   The :file:`std_msgs` package, widely used in ROS1, is deprecated in ROS2 since Foxy. The :file:`example_interfaces` somewhat takes its place, but the recommended practice is to create "semantically meaningful message types". They might remove both or either of these in future versions, so do not use them.

As an example, let us take a look into the :file:`example_interfaces` package, containing, as the name implies, example interface types. We can do so with 
 
.. code:: console

    ros2 interface package example_interfaces
    
which returns

.. code:: console

    example_interfaces/msg/String
    example_interfaces/srv/AddTwoInts
    example_interfaces/srv/SetBool
    example_interfaces/msg/UInt8
    example_interfaces/msg/Int64MultiArray
    example_interfaces/msg/Byte
    example_interfaces/msg/Float32
    example_interfaces/msg/Int64
    example_interfaces/msg/UInt32MultiArray
    example_interfaces/msg/Int32MultiArray
    example_interfaces/msg/Empty
    example_interfaces/msg/Float32MultiArray
    example_interfaces/msg/Int16MultiArray
    example_interfaces/action/Fibonacci
    example_interfaces/msg/UInt16MultiArray
    example_interfaces/msg/Int8MultiArray
    example_interfaces/msg/Bool
    example_interfaces/msg/ByteMultiArray
    example_interfaces/msg/MultiArrayLayout
    example_interfaces/msg/UInt8MultiArray
    example_interfaces/msg/UInt16
    example_interfaces/msg/Int16
    example_interfaces/msg/Int8
    example_interfaces/msg/MultiArrayDimension
    example_interfaces/msg/Char
    example_interfaces/msg/Float64
    example_interfaces/srv/Trigger
    example_interfaces/msg/UInt64
    example_interfaces/msg/WString
    example_interfaces/msg/Int32
    example_interfaces/msg/Float64MultiArray
    example_interfaces/msg/UInt64MultiArray
    example_interfaces/msg/UInt32

Messages
--------

For example, let's say that we are interested in looking up the contents of :file:`example_interfaces/msg/String`. We can do so with :program:`ros2 interface show`, like so

.. code:: console

    ros2 interface show example_interfaces/msg/String
    
which returns the contents of the source file used to create this message

.. code-block:: yaml
    :emphasize-lines: 5

    # This is an example message of using a primitive datatype, string.
    # If you want to test with this that's fine, but if you are deploying
    # it into a system you should create a semantically meaningful message type.
    # If you want to embed it in another message, use the primitive data type instead.
    string data

Basically, the comments help to emphasize that interface types with too broad meaning are unloved in ROS2. Given that these example interfaces are either unsupported or only loosely supported, do not rely on them.

The real content of the message file is :code:`string data`, showing that it contains a single string called :code:`data`. Using :code:`ros2 interface show` on other example interfaces, it is easy to see how to build interesting message types to fit our needs.

Services
--------

In the case of a service, let's look up the contents of :file:`example_interfaces/srv/AddTwoInts`.

We run

.. code:: console

    ros2 interface show example_interfaces/srv/AddTwoInts
    
that results in
    
.. code-block:: yaml
   :emphasize-lines:  3

   int64 a
   int64 b
   ---
   int64 sum

Notice that the :code:`---` is what separates the :code:`Request`, above, from the :code:`Response` below. Anyone using this service would expect that the result would be :math:`sum = a + b`, but this logic needs to be implemented on the Node. The service itself is just a way of bidirectional communication.
