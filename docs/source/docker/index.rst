Happy days with :program:`docker`
=================================


.. include:: ../the_topic_is_under_heavy_construction.rst

Installation
------------
.. note::

   This is a quick installation script for a fresh system.

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

.. code-block:: console

    docker run hello-world

SAS testing
-----------

For the purposes of this illustration we will use the image ``murilomarinho/sas``.

Docker run interactively
++++++++++++++++++++++++

Firstly it would be easier to tackle docker in simple commands before tackling complex
scenarios.

We can start with the simple

.. code-block:: console

   docker run -it --rm murilomarinho/sas

Where the flags

#. ``-it`` will open an interactive shell and
#. ``--rm`` will remove all changes and return the image to its fresh initial state after we're done.

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

            export ROS_DOMAIN_ID=1
            ros2 topic echo /chatter

.. tip::

    You can close an interactive session on a container by typing on that terminal.

    .. code-block::

        exit

.. note::

   In the container we have ``ROS_DOMAIN_ID=1``. If this is modified in the container, this must
   be also modified in the host or they will not find and communicate with each other.

Notice that host and container will communicate without any issues. Changing the network settings of the
docker container may cause this to stop working.

Docker compose
++++++++++++++

If your host does not have :program:`ROS2` you can also have multiple containers communicating with each
other without any direct involvement of the host. For instance with the following compose file named
:file:`compose.yml` below.

.. literalinclude:: scripts/composer/simple_example/compose.yml
   :language: yaml

In the folder where :file:`compose.yml` exists, we do

.. code-block::

    docker compose up --force-recreate -V

.. tip::

    The ``--force-recreate`` and ``-V`` are an overkill for this toy problem but that is the best equivalent
    around for docker compose to do something similar to ``--rm`` as in :program:`docker`. In tutorials it
    is convenient to always start fresh.

    Notice that this will cause data to be lost in the container when it's closed.

This will show the output of the two images communicating over :program:`ROS2`. Notice that there is no input
from the host and no complicated network setup involved.

You can stop the process with ``CTRL+C``. Note that we are using ``stop_signal: SIGINT`` in the docker compose
file because otherwise it will send ``SIGTERM``. In this toy example this is not an issue, but this is a major
issue for nodes that control real resources. This can mean that a robot will not be safely disconnected before
the container is destroyed leaving it in an undetermined state. For the same reason we have ``stop_grace_period``
to give the container enough time to close safely. Depending on your resource you'd prefer to have a larger
value so that it is not escalated into a ``SIGTERM`` or ``SIGKILL`` before your container had enough time to
sort out its shutdown procedure.

Docker container in a realtime kernel
-------------------------------------

TODO

Install PREEMPT_RT
++++++++++++++++++

.. seealso::

    #. https://ubuntu.com/real-time
    #. https://canonical-ubuntu-pro-client.readthedocs-hosted.com/en/latest/howtoguides/enable_realtime_kernel/

.. code-block:: console

    sudo pro attach
    sudo apt update && sudo apt install ubuntu-advantage-tools
    sudo pro enable realtime-kernel

.. tip::

    You can check if a process has properly been elevated to ``SCHED_FIFO`` with the following command.

    .. code-block:: console

        ps -eLfc | grep FF

Tips and troubeshooting
-----------------------

This documents part of my own misunderstandings when getting used to docker (which is an ongoing process) and other difficulties that are
contributed by others.

The standard shell is not interactive
+++++++++++++++++++++++++++++++++++++

.. seealso::

    https://github.com/traefik/traefik/issues/12253#issuecomment-3515555316

Error type

    com.github.dockerjava.api.exception.DockerException: Status 400: client version 1.24 is too old. Minimum supported API version is 1.44, please upgrade your client to a newer version. To fix it, change the project interpreter or check settings.

.. code-block::

    sudo systemctl edit docker.service

Add this line.

.. code-block::

    [Service]
    Environment=DOCKER_MIN_API_VERSION=1.24

.. code-block::

    systemctl restart docker

The standard shell is not interactive
+++++++++++++++++++++++++++++++++++++

When we start using :program:`ROS2` we get used to rely on `~/.bashrc` to define
environment variables and sometimes aliases.

Unless explicitly said with flags such as ``-it``, docker does not run
bash in interactive mode. This means that it will not execute anything in :file:`~/.bashrc`.
It does not matter what you add, because the first few lines of :file:`~/.bashrc` check if
the shell is interactive and return if it's not.

This example image that we use will run ``source /etc/bash_env`` in noninteractive shells.
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

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>CustomUdpTransport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="participant_profile" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>CustomUdpTransport</transport_id>
                </userTransports>

                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>

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
