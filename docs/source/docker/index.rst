Happy days with :program:`docker`
=================================

.. include:: ../the_topic_is_under_heavy_construction.rst

For a basic treatise see https://uomresearchit.github.io/docker-introduction/.

This tutorial is about the specifics of ``ROS2`` and ``sas`` when using docker on Ubuntu.

Installation
------------
.. note::

   Information obtained from the main documentation for Ubuntu.
   https://docs.docker.com/engine/install/ubuntu/

.. literalinclude:: scripts/install_docker.sh
   :language: bash
   :lines: 5-

.. warning::

    It's best not to call ``sudo docker ...`` to test it yet. If you're adding your user to the ``docker`` group,
    running ``docker`` as ``sudo`` once will change the access properties of the socket. It's best to test it after
    adding your user to the ``docker`` group.

Adding user to docker group
---------------------------

.. warning::

   Adding a user to the ``docker`` group is a security concern. The users can easily share
   protected volumes and wipe out a whole shared computer. There does not seem to be a
   viable alternative currently so the system manager should be aware of this when adding
   users to the ``docker`` group.

   Further, users' home folders should probably be encrypted to prevent private
   projects and data from being readable by unauthorised users.


You can add the user to ``docker`` group by downloading this helper script

.. code-block:: console

    mkdir -p ~/ros2_tutorial_workspace/docker
    cd ~/ros2_tutorial_workspace/docker
    curl -OL https://raw.githubusercontent.com/mmmarinho/ROS2_Tutorial/refs/heads/main/docs/source/docker/scripts/set_docker_user.sh
    chmod +x set_docker_user.sh

then

.. code-block:: console

    cd ~/ros2_tutorial_workspace/docker
    ./set_docker_user.sh $USER

.. important::

    A reboot of the system is in general needed so that the group addition is propagated when you open a new terminal.

If you're curious, here are the main contents of the script.

.. literalinclude:: scripts/set_docker_user.sh
   :language: bash
   :lines: 16-


Basic testing
-------------

In a terminal window, do the following.

.. code-block:: console

    docker run hello-world

If executed for the first time in the machine, it should return something similar to the following. If the image already exists in the system, it will not be pulled.

.. code-block:: console

    Unable to find image 'hello-world:latest' locally
    latest: Pulling from library/hello-world
    58dee6a49ef1: Pull complete 
    c3bdf82c34d1: Download complete 
    Digest: sha256:0e760fdfbc48ba8041e7c6db999bb40bfca508b4be580ac75d32c4e29d202ce1
    Status: Downloaded newer image for hello-world:latest
    
    Hello from Docker!
    This message shows that your installation appears to be working correctly.
    
    To generate this message, Docker took the following steps:
     1. The Docker client contacted the Docker daemon.
     2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
        (arm64v8)
     3. The Docker daemon created a new container from that image which runs the
        executable that produces the output you are currently reading.
     4. The Docker daemon streamed that output to the Docker client, which sent it
        to your terminal.
    
    To try something more ambitious, you can run an Ubuntu container with:
     $ docker run -it ubuntu bash
    
    Share images, automate workflows, and more with a free Docker ID:
     https://hub.docker.com/
    
    For more examples and ideas, visit:
     https://docs.docker.com/get-started/

SAS testing
-----------

For the purposes of this illustration we will use the image ``murilomarinho/sas:jazzy``.

Docker run interactively
++++++++++++++++++++++++

Firstly it would be easier to tackle docker in simple commands before tackling complex
scenarios.

We can start with the simple command below.

.. code-block:: console

   docker run -it --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID murilomarinho/sas:jazzy

The flags

#. ``-it`` will open an interactive shell and
#. ``--rm`` will remove all changes and return the image to its fresh initial state after we're done.
#. ``-e`` will pass environment variables to the container.

For this example, we will have ``ROS_DOMAIN_ID=$ROS_DOMAIN_ID``. This means that the container will have the same ``ROS_DOMAIN_ID`` as the host.
This is usually sufficient, but keep that in mind in case something is not communicating as expected.

The terminal in which you ran the ``docker run`` command should now be logged inside the container.
The computer from which you ran is called the host. We will use this terminology to help explain where
each command should be used.

Then, in the container and in the host we do as follows.

.. tab-set::

    .. tab-item:: Container

        .. code-block:: console

            ros2 run demo_nodes_cpp talker

    .. tab-item:: Host

        .. code-block:: console

             ros2 topic echo /chatter

.. tip::

    You can close an interactive session on a container by typing on that terminal.

    .. code-block::

        exit


Notice that host and container will communicate without any issues. Changing the network settings of the
docker container may cause this to stop working, so make sure you know what you're doing.

Docker compose
++++++++++++++

If your host does not have :program:`ROS2` you can also have multiple containers communicating with each
other without any direct involvement of the host. For instance with the following compose file named
:file:`compose.yml` below.

:download:`simple_example/compose.yml <scripts/simple_example/compose.yml>`

.. literalinclude:: scripts/compose/simple_example/compose.yml
   :language: yaml

In the folder where :file:`compose.yml` exists, we do

.. code-block::

    docker compose pull

to make sure the image we have locally is indeed the latest.

.. warning::

    If the :file:`compose.yml` refers to a :file:`Dockerfile` instead of an image, then ``pull`` will not have the
    intended effect. Instead, we do

    .. code-block::

        docker compose build --pull

    .. tip::

        The ``--pull`` flag is useful most of the time. This will guarantee that any images that your compose file
        depends on will be pulled to their latest version. This can save a lot of time.

        The ``--no-cache`` flag is useful when things changed in the image but did not trigger a re-build of that part of
        the cache. Most of the time you won't need to use it and instead it will cost you additional time.

        You can combine them like so.

        .. code-block::

            docker compose build --pull --no-cache

This will show the output of the two containers communicating over :program:`ROS2`. Notice that there is no input
from the host and no complicated network setup involved. Each container is called a service.

You can stop the process with ``CTRL+C``. Note that we are using ``stop_signal: SIGINT`` in the docker compose
file because otherwise it will send ``SIGTERM``. In this toy example this is not an issue, but this is a major
issue for nodes that control real resources. This can mean that a robot will not be safely disconnected before
the container is destroyed leaving it in an undetermined state. For the same reason we have ``stop_grace_period``
to give the container enough time to close safely. Depending on your resource you'd prefer to have a larger
value so that it is not escalated into a ``SIGTERM`` or ``SIGKILL`` before your container had enough time to
sort out its shutdown procedure.

You will note that ``SIGINT`` sent this way might not kill one or another service.
I haven't found too much discussion about the topic in the :program:`ROS2` planet, but the gist of it is that we are not
running :program:`ROS2` commands directly in these examples. We are running a :program:`bash` shell, which will configure itself
with the :program:`ROS2` workspace, then running ``ros2 run`` inside it. Therefore, the ``SIGINT`` is sent to
:program:`bash` first which will forward that signal to whatever it is running below. If the program ignores it,
then it will be ignored.

Although in :program:`sas` and in these tutorials there is much effort to guarantee nodes finish cleanly with
a ``SIGINT``, there will be many examples online that do not do this correctly.

.. seealso::

    You might be interested in reading about the *wait and cooperative exit* implemented in :program:`bash` to
    save yourself from the headaches of edge cases.

    https://mywiki.wooledge.org/SignalTrap



Docker container in a realtime kernel
-------------------------------------

.. seealso::

   Real-life example: https://github.com/MarinhoLab/sas_ur_control_template

Making things work on a realtime kernel is a relatively simple task.

#. Install the realtime kernel on the host.
#. Set up the container to take advantage of the realtime capabilities.

Install ``PREEMPT_RT`` on the host
++++++++++++++++++++++++++++++++++

.. seealso::

    #. https://ubuntu.com/real-time
    #. https://canonical-ubuntu-pro-client.readthedocs-hosted.com/en/latest/howtoguides/enable_realtime_kernel/

.. important::

   From Ubuntu 26.04, the following command will suffice. Ubuntu pro is no longer needed.

   .. code-block:: console

       sudo apt update && sudo apt install ubuntu-realtime

For ubuntu 24.04, we do the following.

.. code-block:: console

    sudo pro attach
    sudo apt update && sudo apt install ubuntu-advantage-tools
    sudo pro enable realtime-kernel

.. tip::

    You can check if a process has properly been elevated to ``SCHED_FIFO`` with the following command.

    .. code-block:: console

        ps -eLfc | grep FF

The :file:`compose.yml`
+++++++++++++++++++++++

For real-time performance, additional capabilities must be given to the container.

:download:`realtime_example/compose.yml <scripts/realtime_example/compose.yml>`

.. literalinclude:: scripts/compose/realtime_example/compose.yml
   :language: yaml

For this example, the relevant parameters are ``cap_add``, ``rtprio``, and ``rttime``. The first one is to add the capability of setting process `niceness <https://manpages.ubuntu.com/manpages/focal/en/man1/nice.1.html>`_. Then,
the other two are related to the realtime priorities. 

The compose file does not make anything realtime. For a realtime thread you will have to set up the thread scheduling properly to ``SCHED_FIFO`` or ``SCHED_RR``
We can run one example doing so in ``sas_core`` is shown below.

.. rli:: https://raw.githubusercontent.com/SmartArmStack/sas_core/refs/heads/jazzy/src/examples/sas_clock_sched_fifo_example.cpp
    :language: cpp
    :lines: 25-

With the :file:`compose.yml` above, we do the following.

.. code-block:: console

    docker compose up

Which should output something similar to the following.

.. code-block:: console

    [+] up 1/1
     ✔ Container realtime_example-realtime-1 Recreated                                                                                                                                                                                                                                                             0.3s
    Attaching to realtime-1
    realtime-1  | **************************************************************************
    realtime-1  | sas::Clock (c) Murilo M. Marinho (murilomarinho.info) 2016-2026 LGPLv3
    realtime-1  | **************************************************************************
    realtime-1  | Statistics for the entire loop
    realtime-1  |   Mean computation time: 3.42e-07
    realtime-1  |   Mean idle time: 0.000999767
    realtime-1  |   Mean effective thread sampling time: 0.00100012
    realtime-1  |   Overrun count: 0
    realtime-1  |
    realtime-1 exited with code 0

Note that if the correct capabilities are not available or if the correct kernel is not installed, even such a simple
example can show clock overruns.

See below one run after removing ``cap_add``, ``rtprio``, and ``rttime``.

.. code-block:: console

    [+] up 1/1
     ✔ Container realtime_example-realtime-1 Recreated                                                                                                                                                                                                                                                             0.1s
    Attaching to realtime-1
    realtime-1  | **************************************************************************
    realtime-1  | sas::Clock (c) Murilo M. Marinho (murilomarinho.info) 2016-2026 LGPLv3
    realtime-1  | **************************************************************************
    realtime-1  | Failed to setschedparam: Operation not permitted
    realtime-1  | Statistics for the entire loop
    realtime-1  |   Mean computation time: 7.02e-07
    realtime-1  |   Mean idle time: 0.00106458
    realtime-1  |   Mean effective thread sampling time: 0.00106529
    realtime-1  |   Overrun count: 3
    realtime-1  |
    realtime-1 exited with code 0

Tips and troubleshooting
------------------------

This documents part of my own misunderstandings when getting used to docker (which is an ongoing process) and other difficulties that are
contributed by others.

Cleaning things up
++++++++++++++++++

Related information: https://docs.docker.com/engine/manage-resources/pruning/.

.. caution::

    This will remove all containers, cache, and volumes. Check the instructions carefully before moving forward.

.. code-block:: console

    docker system prune --all --volumes

The standard shell is not interactive
+++++++++++++++++++++++++++++++++++++

When we start using :program:`ROS2` we get used to rely on `~/.bashrc` to define
environment variables and sometimes aliases.

Unless explicitly said with flags such as ``-it``, docker does not run
bash in interactive mode. This means that it will not execute anything in :file:`~/.bashrc`.
It does not matter what you add, because the first few lines of :file:`~/.bashrc` check if
the shell is interactive and return if it's not.

This example image that we use runs ``source /etc/bash_env`` in noninteractive shells.
However, it will not run for interactive shells and aliases that we define will not work.
Noninteractive shells by default do not expand aliases.

The "easiest" solution is

#. Set your :file:`Dockerfile` to source ``/etc/bash_env``
#. Add ``source source /etc/bash_env`` to your :file:`~/.bashrc` exactly once.

Stop with the ``--net=host`` for everything
+++++++++++++++++++++++++++++++++++++++++++

.. seealso::

    #. https://robotics.stackexchange.com/questions/98161/ros2-foxy-nodes-cant-communicate-through-docker-container-border
    #. https://github.com/rosblox/ros-template
    #. https://github.com/eProsima/Fast-DDS/issues/1750

Although this command can help in some situations it is not recommended unless strictly
necessary. It can for instance cause :program:`ros2` to no longer be able to communicate between
host and container.

The reason for this is that the current version of :program:`ROS2` is based on ``FASTRTPS``. When
``--net=host`` is defined, ``FASTRTPS`` tries to communicate via shared memory.

There are two ways of solving this issue.

#. Adjust the docker/linux settings via various mechanisms to expose the shared memory.
#. Adjusting the ``FASTRTPS`` profile so that the nodes communicate via ``UDP``.

The main difficulty with the first solution is when the user calling the container is not ``root``. There are many
cases in which this won't be true, for instance for shared systems in which docker users are simply added to the
``docker`` group. In addition, calling ``ros2`` programs with ``sudo`` can cause many issues with permissions and
further complicate the use of ``ros2`` in the host. There are `smart solutions <https://github.com/rosblox/ros-template>`_ to aid
in setting up the user properly. But I would only recommend going this route if the shared memory communication
is paramount for performance which usually is not the case.

Alternatively, my suggested solution is to modify the container to force it to communicate over UDP. This has been
well described in `this issue <https://github.com/eProsima/Fast-DDS/issues/1750>`_.

This can be done by copying the contents below in the file :file:`~/ros2_tutorial_workspace/docker/fastrtps_profile.xml`.

.. literalinclude:: fastrtps_profile.xml
   :language: xml

Then, in the container, you can set before the nodes.

.. code-block:: console

    export FASTRTPS_DEFAULT_PROFILES_FILE="~/ros2_tutorial_workspace/docker/fastrtps_profile.xml"

Sad notes on rootless :program:`docker`
+++++++++++++++++++++++++++++++++++++++

Although ideally having a rootless docker environment is what would be expected
in a situation with shared computers and equipment the experience as of now adds
a lot of complexity. In these notes I will focus on trying to reduce risks by
not adding more docker directives or requirements than those strictly need for
things to work but with rootless docker there was a consistent difficulty with
things such as.

#. ROS2 discovery. Settings become more difficult although can be circumvented.
#. Port exposure and external access. Some robotic systems need to open reverse sockets with the host and networking becomes more difficult that it should otherwise be in rootless docker.
