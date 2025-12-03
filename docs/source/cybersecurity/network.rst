Network topologies
==================

.. include:: ../the_topic_is_under_heavy_construction.rst

Case 1
++++++

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    architecture-beta
        group api(cloud)[Internet]

        service internet(internet)[Internet] in api
        service computer(disk)[Computer] in api
        service robot1(disk)[Robot 1] in api
        service robot2(disk)[Robot 1] in api
        junction junctionCenter1 in api
        junction junctionCenter2 in api
        junction junctionCenter3 in api

        internet:R -- L:junctionCenter1
        junctionCenter1:B -- T:robot1
        junctionCenter2:B -- T:robot2
        junctionCenter3:B -- T:computer
        junctionCenter2:L -- R:junctionCenter1
        junctionCenter3:L -- R:junctionCenter2

Case 3
++++++

.. mermaid::

    %%{init: { "theme" : "dark" }}%%
    architecture-beta
        group api(server)[Intranet]

        service internet(material-symbols:robot-2)[Enterprise Intranet] in api
        service computer(logos:aws-lambda)[Computer] in api
        service robot1(disk)[Robot 1] in api
        service robot2(disk)[Robot 1] in api
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

        service computer(disk)[Computer] in api
        service router(server)[Switching hub] in api
        service robot1(disk)[Robot 1] in api
        service robot2(disk)[Robot 1] in api
        junction junctionCenter1 in api
        junction junctionCenter2 in api


        computer:R -- L:router
        router:R -- L:junctionCenter1
        junctionCenter1:B -- T:robot1
        junctionCenter2:B -- T:robot2
        junctionCenter2:L -- R:junctionCenter1


