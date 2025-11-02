Using :file:`tf2`
=================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. note::

    In :program:`ROS2`, the package for ``tf2`` is called ``tf2_ros``.

.. hint::

    An official ``tf2`` user guide is available here https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html.

Perhaps the most important benefit of ``tf2`` is how it connects with other :program:`ROS2` parts, such as ``rviz2``.

Create the package
------------------

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_tf2 \
    --build-type ament_python \
    --dependencies geometry_msgs tf2_ros

.. dropdown:: ros2 pkg create output

    .. code-block:: console

        going to create a new package
        package name: python_package_that_uses_tf2
        destination directory: ~/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['geometry_msgs', 'tf2_ros']
        creating folder ./python_package_that_uses_tf2
        creating ./python_package_that_uses_tf2/package.xml
        creating source folder
        creating folder ./python_package_that_uses_tf2/python_package_that_uses_tf2
        creating ./python_package_that_uses_tf2/setup.py
        creating ./python_package_that_uses_tf2/setup.cfg
        creating folder ./python_package_that_uses_tf2/resource
        creating ./python_package_that_uses_tf2/resource/python_package_that_uses_tf2
        creating ./python_package_that_uses_tf2/python_package_that_uses_tf2/__init__.py
        creating folder ./python_package_that_uses_tf2/test
        creating ./python_package_that_uses_tf2/test/test_copyright.py
        creating ./python_package_that_uses_tf2/test/test_flake8.py
        creating ./python_package_that_uses_tf2/test/test_pep257.py

        [WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
        It is recommended to use one of the ament license identifiers:
        Apache-2.0
        BSL-1.0
        BSD-2.0
        BSD-2-Clause
        BSD-3-Clause
        GPL-3.0-only
        LGPL-3.0-only
        MIT
        MIT-0

References
----------

I have not been an avid user of :code:`tf2` myself.
I can somewhat see the appeal at an attempt to abstract away some of the mathematics of pose transformations,
but perhaps the most important aspects are how it connects with other :program:`ROS2` parts, such as ``rviz2``.
- https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
- https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
