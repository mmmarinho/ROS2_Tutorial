How is Murilo using :program:`docker`
=====================================

.. important::

    This section is a working draft.

    It is an Ubuntu 24.04 host tutorial. It might work in other host systems in some cases
    but the support is finicky for more advanced elements.

Installation
------------
.. note::

   This is a quick installation script for a fresh system.

Information obtained from the main documentation for Ubuntu.
https://docs.docker.com/engine/install/ubuntu/

.. literalinclude:: scripts/install_docker.sh
   :language: bash
   :lines: 5-

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

If you're curious, here are the main contents of the script.

.. literalinclude:: scripts/set_docker_user.sh
   :language: bash
   :lines: 16-

Basic testing
-------------

For the purposes of this illustration we will use the image 
``murilomarinho/sas``.

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
docker container may cause this to stop working. For instance, a very popular setting is to use
``--net=host``. This is not recommended unless strictly necessary because it will generate issues
with :program:`ROS2` networking.

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

Common mistakes
---------------

This documents part of my own misunderstandings when getting used to docker (which is an ongoing process) and other difficulties that are
contributed by others.

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

#. Set your :file:`Dockerfile` to source /etc/bash_env`
#. Add ``source source /etc/bash_env`` to your :file:`~/.bashrc` exactly once.

--net=host
++++++++++

Although this command can help in some situations it is not recommended unless strictly
necessary. It can for instance cause :program:`ros2` to no longer be able to communicate between
host and container. There are workarounds but those further expose resources that should
only be exposed if strictly required.

Notes on rootless docker
++++++++++++++++++++++++

.. note::

   I am a strong proponent of this capability and will be happy to use it when
   it's working in a less demanding way.

Although ideally having a rootless docker environment is what would be expected
in a situation with shared computers and equipment the experience as of now adds
a lot of complexity. In these notes I will focus on trying to reduce risks by
not adding more docker directives or requirements than those strictly need for
things to work but with rootless docker there was a consistent difficulty with
things such as.

#. ROS2 discovery. Settings become more difficult although can be circumvented.
#. Port exposure and external access. Some robotic systems need to open reverse sockets with the host and networking becomes more difficult that it should otherwise be in rootless docker.
