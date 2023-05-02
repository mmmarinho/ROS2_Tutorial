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

1. Go to :menuselection:`File --> Open...` and browse to our workspace folder :file:`~/ros2_tutorial_workspace`
2. Right-click the folder :file:`install` and choose :menuselection:`Mark Directory as --> Excluded`. Do the same for :file:`build` and :file:`log`

Your project view should look like so

.. image:: ../images/pycharm_project.jpg
   :align: center

Running a Node from :code:`PyCharm`
-----------------------------------

With the project correctly configured, you can

1. move to :menuselection:`src --> python_package_with_a_node --> python_package_with_a_node`.
2. double (left) click :program:`sample_python_node.py` to open the source code, showing the contents of the Node. It is minimal to the point that it doesn't have anything related to :program:`ROS` at all.

.. code:: python

   def main():
      print('Hi from python_package_with_a_node.')


   if __name__ == '__main__':
      main()

3. right click :program:`sample_python_node.py` and choose :menuselection:`Run sample_python_node`

It will output in :code:`PyCharm`'s console

.. code:: bash
   
    Hi from python_package_with_a_node.
    
.. note:: 

   You should extensively use the Debugger in :code:`PyCharm` when developing code. If you're still adding :code:`print` functions to figure out what is wrong with your code, now is the opportunity you always needed to stop doing that and join the adult table.

.. note::

   You can read more about debugging with :code:`PyCharm` at the `official documentation <https://www.jetbrains.com/help/pycharm/debugging-your-first-python-application.html#where-is-the-problem>`_
