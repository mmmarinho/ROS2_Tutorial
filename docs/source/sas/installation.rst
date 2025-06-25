Overview
========

.. warning::

   These instructions are for Ubuntu 24.04 and ROS2 Jazzy. You might be able to make this run in other settings,
   but I am currently unable to provide support for those.

Docker image
------------

.. note::

   Information is centralised in https://smartarmstack.github.io.

A docker image with all `sas` software is available as follows

.. code-block::

  docker run murilomarinho/sas_ros_jazzy:latest

From source
-----------

Please see the information at

https://smartarmstack.github.io/installation

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
