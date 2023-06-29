.. _Editing Python source:

Editing Python source (with :program:`PyCharm`)
===============================================

There are near-infinite ways to manage your Python code and, for this tutorial, we will use :program:`PyCharm`. Namely, the free community version.

Installing :program:`PyCharm`
----------------------------

:program:`PyCharm` is a great program for managing one's Python sources that is frequently updated and has a free edition. However, precisely because it is frequently updated, there is no way for this tutorial to keep up with future changes.

What we will do, instead, is to download a specific version of :program:`PyCharm` for these tutorials, so that its behavior/looks/menus are predictable. If you'd prefer using the shiniest new version, be sure to wear sunglasses and not stare directly into the light.

Run

.. code :: console

   cd ~
   mkdir ros2_workspace_pycharm
   cd ros2_workspace_pycharm
   wget https://download.jetbrains.com/python/pycharm-community-2023.1.1.tar.gz
   tar -xzvf pycharm-community-2023.1.1.tar.gz
   
Create an alias for :program:`pycharm_ros2`
-------------------------------------------

.. note:
   Starting :program:`PyCharm` from the terminal has the added benefit of easily recognizing our ROS2, as long as it has been started from a properly sourced terminal. 

To simplify the use of this version of :program:`PyCharm`, let us create a :program:`bash` alias for it. 

.. code :: console

   echo "# Alias for PyCharm, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
   echo "alias pycharm_ros2=~/ros2_workspace_pycharm/pycharm-community-2023.1.1/bin/pycharm.sh" >> ~/.bashrc
   source ~/.bashrc
   
Then, you can run :program:`PyCharm` with

.. code :: console

    pycharm_ros2

.. info::
   We use :program:`pycharm_ros2` instead of :program:`pycharm` on purpose. This prevents our alias from misbehaving if you have another version of :program:`PyCharm` installed.
