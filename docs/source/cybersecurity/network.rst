Network topologies
==================

.. include:: ../the_topic_is_under_heavy_construction.rst

There is plenty of software that can be used to protect your network. One of these is :program:`ufw`, the `uncomplicated
firewall <https://en.wikipedia.org/wiki/Uncomplicated_Firewall>`_. These are good to some extent and I suppose will
always be part of the security suite of companies and institutions.

Also, as part of :program:`ROS2` there is the concept of `ROS_DOMAIN_ID <https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html>`_.
Although this concept exists, it should not be confused with a security measure. Each participant in the network can
easily switch to another ``ROS_DOMAIN_ID`` without authentication or central management. It can be seen as merely a
local filter.

An aspect that is often ignored in robotics labs are the physical network topologies, which are important for safety.
This safety is not only for cybersecurity reasons. Yes, your robot's computer can be attacked if it's exposed. However,
in development environments, you might inadvertently move someone else's robot. In these scenarios it will be much
more efficient to physically isolate the robotic setup if many devices in the network can be shared with users with
varying levels of network understanding.

This section will be based on my experience with multiple robotic systems. Two representative examples are :footcite:p:`Marinho2020`
and :footcite:p:`Marinho2024`.

Case 1 - No isolation
+++++++++++++++++++++

A common network architecture in small companies and laboratories is shown below.

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    architecture-beta
        group api(internet)[Internet]

        service internet(material-symbols:router-outline)[Enterprise Intranet] in api
        service computer(material-symbols:computer-outline)[Computer] in api
        service robot1(material-symbols:robot-2-outline)[Robot 1] in api
        service robot2(material-symbols:robot-2-outline)[Robot 1] in api
        junction junctionCenter1 in api
        junction junctionCenter2 in api
        junction junctionCenter3 in api

        internet:R -- L:junctionCenter1
        junctionCenter1:B -- T:robot1
        junctionCenter2:B -- T:robot2
        junctionCenter3:B -- T:computer
        junctionCenter2:L -- R:junctionCenter1
        junctionCenter3:L -- R:junctionCenter2

In it, each computer and robot is directly connected to the internet. This setup is rather tempting because from the
point-of-view of the robotics software developer they might want to have as much freedom as possible to develop their
software as quickly as possible. Depending on the internet services they want to provide, they might even `DMZ <https://en.wikipedia.org/wiki/DMZ_(computing)>`_
or forward ports that give straight access to local computers. That is a huge security concern. If you leave port 22 open,
it won't take long for someone to try to hack into your machine from somewhere in the world.

This setup leads to a high level of risk. As mentioned in the previous section, although you *might* be able to keep
your computers up-to-date, the same is not usually possible for the robots' control computers. They could have vulnerabilities
such as unpatched security bugs that will leave them exposed. In addition, the robots' might have been left with their
original credentials and network settings, without a firewall. This means that an attacker could easily ssh into the robot.

Although from a robotics software developer point-of-view you might think that they might have little to gain from accessing a robot
computer, that can easily be the first door into any other resource in the network. Just because the computer is attached
somehow to a robot it does not make it less of a computer. It just, usually, makes it easier to exploit the computer.

Case 2 - Subnet isolation
+++++++++++++++++++++++++

A somewhat better network architecture is shown below, because there is one extra layer of isolation. The main difference
here is that different parts of the company have their own subnets.

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    architecture-beta
        group api(server)[Intranet]

        service internet(material-symbols:router-outline)[Enterprise Intranet] in api
        service computer(material-symbols:computer-outline)[Computer] in api
        service router(material-symbols:router-outline)[Router] in api
        service robot1(material-symbols:robot-2-outline)[Robot 1] in api
        service robot2(material-symbols:robot-2-outline)[Robot 1] in api
        junction junctionCenter1 in api
        junction junctionCenter2 in api
        junction junctionCenter3 in api

        internet:R -- L:router
        router:R -- L:junctionCenter1
        junctionCenter1:B -- T:robot1
        junctionCenter2:B -- T:robot2
        junctionCenter3:B -- T:computer
        junctionCenter2:L -- R:junctionCenter1
        junctionCenter3:L -- R:junctionCenter2

In this setup, you can imagine as each laboratory having their own router, that further isolate the network. In this case,
not everyone will be able to see everyone else's devices. In this case, that can have many benefits. One of the benefits
is to make sure that things such as ROS2 topics and services do not collide with someone else's.

For instance, suppose that you have your own robot and camera setup. If anyone else anywhere in the same network attempts
to use the same topics and services, you can have a collision and unwanted behavior. You could have someone attempting
to control the wrong robot arm from a distance. Or, the wrong camera stream is used.

Although ``ROS_DOMAIN_ID`` can help to filter out unwanted messages, it is too easy to set the wrong ``ID``. More importantly,
it is expected that people with a minimal understanding of networking would isolate their setup further.

Case 3 - Platform isolation
+++++++++++++++++++++++++++

A possibly sufficient setup for most robotic demonstrators that need isolation is shown below.

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    architecture-beta
        group api(server)[Intranet]

        service internet(material-symbols:router-outline)[Enterprise Intranet] in api
        service computer(material-symbols:computer-outline)[Computer] in api
        service router(material-symbols:router-outline)[Router] in api
        service switch(material-symbols:router-outline)[Switching hub] in api
        service robot1(material-symbols:robot-2-outline)[Robot 1] in api
        service robot2(material-symbols:robot-2-outline)[Robot 1] in api
        junction junctionCenter1 in api
        junction junctionCenter2 in api

        internet:R -- L:router
        router:R -- L:computer
        computer:R -- L:switch
        switch:R -- L:junctionCenter1
        junctionCenter1:B -- T:robot1
        junctionCenter2:B -- T:robot2
        junctionCenter2:L -- R:junctionCenter1

In this setup, you can imagine each robotic demonstrator having their own, isolated, network. This can be easily
achieved physically using a `switching hub <https://en.wikipedia.org/wiki/Network_switch>`_. This type of physical isolation
of interfaces tends to be beneficial in development environments where software infrastructure is often changing.

.. admonition:: References

    .. footbibliography::

