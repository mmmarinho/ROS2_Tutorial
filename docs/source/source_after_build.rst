.. _Always source after you build:

Always source after you build
=============================

When creating new packages or modifying existing ones, many changes will not be visible by the system unless our workspace is re-sourced.

For example, if we try the following in the terminal window we used to first build this example package

.. code :: console

   ros2 run python_package_with_a_node sample_python_node

it will not work and will output

.. code :: console

   Package 'python_package_with_a_node' not found
   
As the workspace grows bigger and the packages more complex, figuring out such errors becomes a considerable hassle. My suggestion is to always source after a build, so that sourcing errors can always be ruled out.

.. code :: console

    cd ~/ros2_tutorial_workspace
    colcon build
    source install/setup.bash

Troubleshooting tips
--------------------

One important tool to assist in case your package is not found is :program:`ros2 pkg list`. It can be called as follows.

.. code :: console

   ros2 pkg list

It will output a large number of packages even for the most basic installations of :program:`ROS2`. If you are looking for
a particular package, you can use :program:`grep` which is more actively used (and explained) in other parts of this tutorial. For instance,
if you are looking for :file:`python_package_with_a_node` you can do as follows.

.. code :: console

   ros2 pkg list | grep the_simplest_python_package

This will either output nothing if the package is not found or it will output the name of the package, as follows.

.. code :: console

   grep the_simplest_python_package

.. admonition:: Hint for the future you

   In rare cases, the workspace can be left in an unclean state in which older build artifacts cause build and runtime issues, such as failed builds and programs that do not seem to match their intended source code. These artifacts might include old files that should have been removed, issues with dependencies, and so on. In those cases, it might be good to remove the :file:`build`, :file:`install`, and :file:`log` folders before rebuilding and re-sourcing. 
  
.. admonition:: Hint for the future you

   It might also be the case that certain packages fail to build after :file:`build`, :file:`install`, and :file:`log` are removed, or that the build only works after :program:`colcon` is called twice in a row. 
   This is usually because the dependencies of the packages in your workspace are poorly configured and, in consequence, ROS2 is not building them in the correct order. If your workspace does not build properly after being cleaned as mentioned above, you must correct its dependencies until it builds properly.
