.. _Editing Python source:

Editing Python source (with :program:`PyCharm`)
===============================================

There are near infinite ways to manage your Python code and, for this tutorial, we will use :program:`PyCharm`. Namely, the free community version.

Installing :program:`PyCharm`
----------------------------

:program:`PyCharm` is a great program for managing one's Python sources that is frequently updated and has a free edition. However, precisely because it is frequently updated, there is no way for this tutorial to keep track of future changes.

What we will do, instead, is to download an specific :program:`PyCharm` version for these tutorials, so that it is predictable. If you'd prefer using the shiniest new version, be sure to wear sunglasses and to not stare directly into the light.

Run

.. code :: console

   cd ~
   mkdir ros2_workspace_pycharm
   cd ros2_workspace_pycharm
   wget https://download.jetbrains.com/python/pycharm-community-2023.1.1.tar.gz
   tar -xzvf pycharm-community-2023.1.1.tar.gz
   
Create an alias for :code:`pycharm_ros2`
----------------------------------------

:program:`PyCharm` will only recognize ROS2 packages if it was started from a terminal that has been sourced with our ROS2 overlay. To cover this requirement and to simplify the use of this specific version of :program:`PyCharm`, let us create a bash alias for it. 

.. code :: console

   echo "# Alias for PyCharm, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
   echo "alias pycharm_ros2=~/ros2_workspace_pycharm/pycharm-community-2023.1.1/bin/pycharm.sh" >> ~/.bashrc
   source ~/.bashrc
   
Then, you can run :program:`PyCharm` with

.. code :: console

    pycharm_ros2
