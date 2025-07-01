Overview
========

.. warning::

   These instructions are for Ubuntu 24.04 and ROS2 Jazzy. You might be able to make this run in other settings,
   but I am currently unable to provide support for those.

.. note::

   Information is centralised in https://smartarmstack.github.io.

Docker image
------------

.. seealso::

   https://smartarmstack.github.io/#docker

A docker image with all `sas` software is available as follows.

.. code-block::

  docker run -it murilomarinho/sas:latest

Installing on a given system
----------------------------

.. seealso::

   https://smartarmstack.github.io/#installation

Setting up PPA
++++++++++++++

.. code-block::

    curl -s --compressed "https://smartarmstack.github.io/smart_arm_stack_ROS2/KEY.gpg" \
    | gpg --dearmor \
    | sudo tee /etc/apt/trusted.gpg.d/smartarmstack_lgpl.gpg >/dev/null
    sudo curl -s --compressed -o /etc/apt/sources.list.d/smartarmstack_lgpl.list \
    "https://smartarmstack.github.io/smart_arm_stack_ROS2/smartarmstack_lgpl.list"
    sudo apt update
    sudo apt-get install ros-jazzy-sas-*

Create the tutorial folder
--------------------------

Below, we make the directory used throughout the tutorial

.. code-block:: console

  mkdir -p ~/sas_tutorial_workspace/src

Developer environment
---------------------

It is recommended to use the latest versions of

1. `PyCharm Community <https://www.jetbrains.com/pycharm/download/?section=linux>`_
2. `QtCreator <https://www.qt.io/download-qt-installer-oss>`_
