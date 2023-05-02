Editing Python source (with :code:`PyCharm`)
============================================

There are near infinite ways to manage your :code:`Python` code and, for this tutorial, we will use :code:`PyCharm`. Namely, the free community version.

Installing :code:`PyCharm`
--------------------------

:code:`PyCharm` is a great program for managing one's :code:`Python` sources that is frequently updated and has a free edition. However, precisely because it is frequently updated, there is no way for this tutorial to keep track of future changes.

What we will do, instead, is to download an specific :code:`PyCharm` version for these tutorials, so that it is predictable. If you'd prefer using the shiniest new version, be sure to wear sunglasses and to not stare directly into the light.

Run

.. code:: bash

   cd ~
   mkdir ros2_workspace_pycharm
   wget https://download.jetbrains.com/python/pycharm-community-2023.1.1.tar.gz
   tar -xzvf pycharm-community-2023.1.1.tar.gz
   
Create an alias for :code:`pycharm_ros2`
----------------------------------------

:code:`PyCharm` will only recognize :code:`ROS2` packages if it was started from a terminal that has been sourced with our ROS2 overlay. To cover this requirement and to simplify the use of this specific version of :code:`PyCharm`, let us create a bash alias for it. 

.. code:: bash

   echo "# Alias for PyCharm, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
   echo "alias pycharm_ros2="~/ros2_workspace_pycharm/pycharm-community-2023.1.1/bin/pycharm.sh" >> ~/.bashrc
   source ~/.bashrc
   
Then, you can run :code:`PyCharm` with

.. code:: bash

    pycharm_ros2
    
More resources on :code:`PyCharm`
---------------------------------

Given that this is not a tutorial on :code:`PyCharm`, please check the official documentation if some of the advice given here seems unclear.

- https://www.jetbrains.com/help/pycharm/working-with-source-code.html
- And the following :code:`Run, debug, test, and deploy` that doesn't have a link.

Using :code:`PyCharm` for :code:`ROS2` sources
----------------------------------------------

With :code:`PyCharm` opened as instructed above, here are a few tips to make your life easier.

1. Go to :code:`File` >> :code:`Open...` and browse to our workspace folder :code:`~/ros2_tutorial_workspace`
2. Right-click the folder :code:`install` and choose :code:`Mark Directory as` >> :code:`Excluded`. Do the same for :code:`build` and :code:`log`

Running a Node from :code:`PyCharm`
-----------------------------------
