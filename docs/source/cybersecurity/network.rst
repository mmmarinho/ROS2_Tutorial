Network topologies
==================

.. include:: ../the_topic_is_under_heavy_construction.rst

Case 1
++++++

A common network architecture in small companies and laboratories is shown below.

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    architecture-beta
        group api(internet)[Internet]

        service internet(router-outline)[Enterprise Intranet] in api
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

Case 3
++++++

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    architecture-beta
        group api(server)[Intranet]

        service internet(router-outline)[Enterprise Intranet] in api
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

Case 2
++++++

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    architecture-beta
        group api(server)[Intranet]

        service internet(router-outline)[Enterprise Intranet] in api
        service computer(material-symbols:computer-outline)[Computer] in api
        service router(router-outline)[Switching hub] in api
        service robot1(material-symbols:robot-2-outline)[Robot 1] in api
        service robot2(material-symbols:robot-2-outline)[Robot 1] in api
        junction junctionCenter1 in api
        junction junctionCenter2 in api

        router:R -- L:internet
        computer:R -- L:router
        router:R -- L:junctionCenter1
        junctionCenter1:B -- T:robot1
        junctionCenter2:B -- T:robot2
        junctionCenter2:L -- R:junctionCenter1


