.. _Gazebo installation:

:program:`Gazebo` Installation
==============================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

The following command will install :program:`Gazebo Harmonic` and all the pairing libraries for :program:`ROS2 Jazzy`. Please do not attempt to install it following another documentation. This could lead to a different version being installed.

.. rli:: https://raw.githubusercontent.com/UoMMScRobotics/SFR_Gazebo/refs/heads/main/install_gazebo.sh
   :language: bash
   :lines: 5-15

Here is the description of the packages we are installing. You can notice that the packages are being used in the commands to add the :program:`Gazebo Harmonic` packages to our :program:`apt` sources. We're just telling :program:`apt` where to find the packages and install those.

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

Which should result in something similar to the following, if the installation went well.
I would recommend strongly against letting the curiosity get the best of you and clicking on the ``3k_shapes.sdf`` given that it's current freezing all my machines.
Other scenes seem to be working in general.

.. image:: images/screen_quickstart.png
    :width: 100%
