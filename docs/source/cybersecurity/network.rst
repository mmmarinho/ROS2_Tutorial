Network topologies
==================

.. include:: ../the_topic_is_under_heavy_construction.rst


https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html

Case 1
++++++

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
software as quickly as possible. Depending on the internet services they want to provide, they might even DMZ

This setup leads to a high level of risk. As mentioned in the previous section, although you *might* be able to keep
your computers up-to-date, the same is not usually possible for the robots' control computers. They could have vulnerabilities
such as unpatched security bugs that will leave them exposed. In addition, the robots' might have been left with their
original credentials. This means that an attacker could easily ssh into the robot.

Although from a robotics software developer you might think that they might have little to gain from accessing a robot
computer, that can easily be the first door into any other resource in the network. Just because the computer is attached
somehow to a robot it does not make it less of a computer just, usually, easier to exploit.

Case 2
++++++

A somewhat better network architecture is shown below, because there is one extra layer of isolation.

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

Case 3
++++++

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


