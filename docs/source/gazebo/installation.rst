.. _Gazebo installation:

:program:`Gazebo` Installation
==============================

.. versionadded:: Jazzy

   This section.

.. include:: ../the_topic_is_under_heavy_construction.rst

The following command will install :program:`Gazebo Harmonic` and all the pairing libraries for :program:`ROS2 Jazzy`.
This is currently the only version known to be compatible with this tutorial.

.. rli:: https://raw.githubusercontent.com/UoMMScRobotics/SFR_Gazebo/refs/heads/main/install_gazebo.sh
   :language: bash
   :lines: 5-15

.. danger::

    If your intention is to follow these tutorials, please do not attempt to install it following any other documentation.
    This could lead to a different version being installed and this tutorial is unlikely to work.


Here is the description of the packages we are installing. You can notice that the packages are being used in the commands to add the :program:`Gazebo Harmonic` packages to our :program:`apt` sources. We're just telling :program:`apt` where to find the packages and install those.

===========================   =================================================================================================================================================
curl_                         Helps download files from the terminal.
lsb-release_                  The lsb_release command is a simple tool to help identify the Linux distribution being used and its compliance with the Linux Standard Base.
gnupg_                        :program:`GnuPG` is an universal crypto engine which can be used directly from a command line prompt, from shell scripts, or from other programs.
===========================   =================================================================================================================================================

Running :program:`Gazebo`
-------------------------

.. note::

    The ``gz sim`` command might not work if you have issues in your network configuration. You might need to enable this through your firewall.

    .. code-block:: console

        sudo ufw allow 10317:10318/udp

    .. danger::

        Obviously do not do this unless you need to, check with only ``gz sim`` first. Exposing ports through the firewall
        might leave your computer exposed to malicious actions.


    Content from ``ROS 2 Networking and Communication``

        https://github.com/UoMMScRobotics/UOMDocumentationForLeoRover/blob/main/Further_reading/Networking.m
        by https://github.com/Https404PaigeNotFound

After installation, :program:`Gazebo Harmonic` can be run with the following command

.. code-block:: console

    gz sim

Which should result in something similar to the following, if the installation went well.

.. danger::

    The ``3k_shapes.sdf`` is current freezing all my machines. Click at your own risk.
    Other scenes seem to be working in general.

.. image:: images/screen_quickstart.png
    :width: 100%

.. _curl: https://curl.se/
.. _lsb-release: https://packages.debian.org/sid/lsb-release
.. _gnupg: https://gnupg.org/download/