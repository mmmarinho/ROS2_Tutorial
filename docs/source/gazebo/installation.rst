.. include:: ../the_topic_is_under_heavy_construction.rst

:program:`Gazebo` quickstart
============================

.. note::

   Here are the `official docs <https://gazebosim.org/docs/harmonic/getstarted/>`_ from the developers of :program:`Gazebo`.

.. _Gazebo installation:

:program:`Gazebo` Installation
------------------------------

The following command will install :program:`Gazebo Harmonic` and all the pairing libraries for :program:`ROS2 Jazzy`.

.. rli:: https://raw.githubusercontent.com/UoMMScRobotics/SFR_Gazebo/refs/heads/main/install_gazebo.sh
   :language: bash
   :lines: 5-15

Here is the description of the packages we are installing. You can notice that the packages are being used in the commands to add the :program:`Gazebo Harmonic` packages to our :program:`apt` sources.

===========================   =================================================================================================================================================
curl_                         Helps download files from the terminal.
`lsb-release`_                The lsb_release command is a simple tool to help identify the Linux distribution being used and its compliance with the Linux Standard Base
gnupg_                        :program:`GnuPG` is an universal crypto engine which can be used directly from a command line prompt, from shell scripts, or from other programs.
===========================   =================================================================================================================================================

Running :program:`Gazebo`
-------------------------

After installation, :program:`Gazebo Harmonic` can be run with the following command

.. code-block:: console

    gz sim


.. _curl: https://curl.se/
.. _`lsb-release`: https://packages.debian.org/sid/lsb-release
.. _gnupg: https://gnupg.org/download/