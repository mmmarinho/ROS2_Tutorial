Troubleshooting :program:`docker`
=================================


.. include:: ../the_topic_is_under_heavy_construction.rst

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

