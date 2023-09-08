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

.. admonition:: Hint for the future you

   In rare cases, the workspace can be left in an unclean state in which older build artifacts cause issues. These might include old files that should have been removed, issues with dependencies and so on. In those cases, it might be good to remove the :file:`build`, :file:`install`, and :file:`log` folders before rebuilding and re-sourcing. 
  
.. admonition:: Hint for the future you

   It might also be the case that certain packages fail to build after :file:`build`, :file:`install`, and :file:`log` are removed, or that the build only works after :program:`colcon` is called twice in a row. 
   This is usually because the dependencies of the packages in your workspace are poorly configured and, in consequence, ROS2 is not building them in the correct order. If your workspace does not build properly after being cleaned as mentioned above, you must correct its dependencies until it builds properly.
