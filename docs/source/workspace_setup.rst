Workspace setup
===============

Similar to how ROS2 files are installed in :code:`/opt/ros/{ROS_DISTRO}` so that you can have several distributions installed simultaneously, you can also have many separate workspaces in your system.

In addition, because files in the :code:`/opt` folder require superuser privileges (for good reasons), having a user-wide workspace is the accepted practice. They call this an **overlay**.

Setting up
----------

In ROS2, a workspace is nothing more than a folder in which all your packages are contained.

No, really, you just need to make a folder, e.g. the one we will use throughout these tutorials.

.. code :: console

   cd ~
   mkdir -p ros2_tutorial_workspace/src
   
It is common practice to have all source files inside the :code:`src` folder, so we will also do so for these tutorials. Nonetheless, it is not a strict requirement.
   
First build
-----------

Regardless of it being a currently empty project, we run :program:`colcon` once to set up the environment and illustrate a few things.
The program :program:`colcon` is the build system of ROS2 and will be described in more detail later.

For now, run

.. code :: console

   cd ~/ros2_tutorial_workspace
   colcon build
   
for which the output will be something similar to

.. code :: console

   Summary: 0 packages finished [0.08s]
   
given that we have an empty workspace, no surprise here.

The folders :code:`build`, :code:`install`, and :code:`log` have been generated automatically by :program:`colcon`. The project structure becomes as follows.

.. code-block:: console
   :emphasize-lines: 2-4
   
   ros2_tutorial_workspace/
   |-- build
   |-- install
   |-- log
   `-- src
    
Inside the :code:`install` folder lie everything in the project that can be accessed by the users.

.. note::

   An easy way to understand if something is not accessible via :program:`ROS2` commands is if they cannot be found
   inside your `install` folder.

Do the following just once, so that all terminal windows automatically source this new workspace for you.

.. code :: console

   echo "# Source the ROS2 overlay, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
   echo "source ~/ros2_tutorial_workspace/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   
However, since our workspace is currently empty, there's not much we can do with it. Let's add some content.
   
