.. include:: ../the_topic_is_under_heavy_construction.rst

.. versionadded:: Jazzy

   Added this section.

Call for Actions: Servers and Clients
=====================================

What about a mixture of :code:`Messages` and :code:`Services`? That is where :code:`Actions` come into play.

We use :code:`Actions` by creating an :code:`ActionServer`. The :code:`ActionServer` will provide an action that can be accessed by one or more :code:`ActionClient`\s.

Similarly to a :code:`Service`, each :code:`Action` should only have a single :code:`ActionServer` that will receive a :code:`Goal` and provide a :code:`Result`.
The :code:`Feedback` topic is XYZ.

Create the package
------------------

We start by creating a package to use the :code:`Service` we first created in :ref:`The service file`.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_the_actions \
    --build-type ament_python \
    --dependencies rclpy package_with_interfaces

.. dropdown:: ros2 pkg create output

   .. code :: console

        going to create a new package
        package name: python_package_that_uses_the_actions
        destination directory: ~/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['rclpy', 'package_with_interfaces']
        creating folder ./python_package_that_uses_the_actions
        creating ./python_package_that_uses_the_actions/package.xml
        creating source folder
        creating folder ./python_package_that_uses_the_actions/python_package_that_uses_the_actions
        creating ./python_package_that_uses_the_actions/setup.py
        creating ./python_package_that_uses_the_actions/setup.cfg
        creating folder ./python_package_that_uses_the_actions/resource
        creating ./python_package_that_uses_the_actions/resource/python_package_that_uses_the_actions
        creating ./python_package_that_uses_the_actions/python_package_that_uses_the_actions/__init__.py
        creating folder ./python_package_that_uses_the_actions/test
        creating ./python_package_that_uses_the_actions/test/test_copyright.py
        creating ./python_package_that_uses_the_actions/test/test_flake8.py
        creating ./python_package_that_uses_the_actions/test/test_pep257.py

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