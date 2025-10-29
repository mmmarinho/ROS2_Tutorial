.. _Always source after you build:

Always source after you build
=============================

When creating new packages or modifying existing ones, many changes will not be visible by the system unless our
workspace is re-built and re-sourced.

For example, if we try the following in the terminal window we used to first build this example package

.. code :: console

   ros2 run python_package_with_a_node sample_python_node

it will not work and will output

.. code :: console

   Package 'python_package_with_a_node' not found
   
As the workspace grows bigger and the packages more complex, figuring out such errors becomes a considerable hassle.
My suggestion is to always source after a build, so that sourcing errors can always be ruled out.

.. code :: console

    cd ~/ros2_tutorial_workspace
    colcon build
    source install/setup.bash

.. note::

    Remember to source this newly built workspace in *every* terminal in which you want to use these packages.
    In these other terminals, you can simply source the workspace.

    .. code :: console

        cd ~/ros2_tutorial_workspace
        source install/setup.bash

Troubleshooting tips
--------------------

In this section I show some troubleshooting tips for common issues related to :program:`colcon build`. You might not understand all of them
at this stage of the tutorial. Check back to this section in case problems arise.

Package not found
+++++++++++++++++

One important tool to assist in case your package is not found is :program:`ros2 pkg list`. It can be called as follows.

.. code :: console

   ros2 pkg list

It will output a large number of packages even for the most basic installations of :program:`ROS2`. If you are looking for
a particular package, you can use :program:`grep` which is more actively used (and explained) in other parts of this tutorial. For instance,
if you are looking for :file:`python_package_with_a_node` you can do as follows.

.. code :: console

   ros2 pkg list | grep python_package_with_a_node

This will either output nothing if the package is not found or it will output the name of the package, as follows.

.. code :: console

   python_package_with_a_node

Fixing a dirty state in your :program:`colcon build`
++++++++++++++++++++++++++++++++++++++++++++++++++++

Sometimes, a problematic build might not go away even with repeated calls to :program:`colcon build`.

The most common cause of this is when, by mistake, a terminal with an active :program:`venv` was used when calling
:program:`colcon build`. The usual error message will look like so.

.. code-block:: console

        Traceback (most recent call last):
          File "/opt/ros/jazzy/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 22, in <module>
            from catkin_pkg.package import parse_package_string
        ModuleNotFoundError: No module named 'catkin_pkg'

To fix this, you must

#. Deactivate the :program:`venv`.
#. Remove the :file:`build`, :file:`install`, and :file:`log` folders.
#. Rebuild and resource in a clean terminal, without a :program:`venv`.

In this tutorial, this would be equivalent to doing

.. caution::

    Remember that :program:`rm` can cause *permanent* loss of data. Please understand the following command and its implications
    *before* executing it.

.. code :: console

    deactivate
    cd ~/ros2_tutorial_workspace
    rm -rf build/ install/ log/
    colcon build
    source install/setup.bash

In rare cases, even without using a :program:`venv`, the workspace can be left in an unclean state in which older build
artifacts cause build and runtime issues, such as failed builds and programs that do not seem to match their intended source code.
These artifacts might include old files that should have been removed, issues with dependencies, and so on.
In this case, removing the :file:`build`, :file:`install`, and :file:`log` folders can be useful.

Dependency issues in the first :program:`colcon build`
++++++++++++++++++++++++++++++++++++++++++++++++++++++

It might also be the case that certain packages fail to build after :file:`build`, :file:`install`, and :file:`log` are removed,
or that the build only works after :program:`colcon` is called twice in a row.

This is usually because the dependencies of the packages in your workspace are poorly configured and, in consequence,
ROS2 is not building them in the correct order. If your workspace does not build properly after being cleaned as mentioned
above, you must correct its dependencies until it builds properly.

This can usually done by verifying if your :file:`package.xml` has the correct dependencies.
