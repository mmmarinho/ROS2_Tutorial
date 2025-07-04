How is Murilo using :program:`docker`
=====================================

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

   Adding a user to the `docker` group is a security concern. The users can easily share
   protected volumes and wipe out a whole shared computer. There does not seem to be a
   viable alternative currently so the system manager should be aware of this when adding
   users to the `docker` group. Further, users' home folders should probably be encrypted to prevent
   projects from being accessible by unauthorised users.

You can either set manually `$USER_TO_ADD` in the script below

.. literalinclude:: scripts/set_docker_user.sh
   :language: bash
   :lines: 13-

or use the script in this folder with the user as the argument.

Notes on rootless docker
------------------------

.. note::

   I am a strong proponent of this capability and will be happy to use it when
   it's working in a less demanding way.

Although ideally having a rootless docker environment is what would be expected
in a situation with shared computers and equipment the experience as of now adds
a lot of complexity. In these notes I will focus on trying to reduce risks by
not adding more docker directives or requirements than those strictly need for
things to work but with rootless docker there was a consistent difficulty with
things such as.

- ROS2 discovery. Settings become more difficult although can be circumvented.
- Port exposure and external access. Some robotic systems need to open reverse
sockets with the host and networking becomes more difficult that it should otherwise
be in rootless docker.
