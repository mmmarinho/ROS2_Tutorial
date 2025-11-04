The support library ``nottf2``
------------------------------

.. include:: ../the_topic_is_under_heavy_construction.rst

.. danger::

    You do not need this support library if you're just creating basic rotations. You can create easy rotations directly using
    ``math.cos``, ``math.sin``, and the class ``Quaternion`` part of ``geometry_msgs``.
    The support library becomes useful if you need to, for instance, multiply quaternions to obtain relative rotations with ``quaternion_multiply``.

In ``rclpy``, ``tf2`` does not (currently?) have quaternion operations. But I got you covered. Kinda. It's more use at your
own risk type of arrangement.

For the library to be properly found in our environment, it needs to be installed in the system's Python.
We can do so as follows.

.. code-block:: console

    python3 -m pip install nottf2 --break-system-packages

.. caution::

    The ``nottf2`` library is provided as-is, with no warranty. Use at your own risk (See ``MIT`` license for more details).

    The creator of ``nottf2``, me, will not accept any blame or liability for any issues, including but limited to broken
    robots or broken code in your assignments. If you feel it's insufficient or unreliable, please create and use your own
    implementation, or someone else's that fits your needs.

Create :program:`ros2` package that uses ``nottf2``
---------------------------------------------------

Let's first create our sample package, as follows, that will depend on ``geometry_msgs``.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_nottf2 \
    --build-type ament_python \
    --dependencies geometry_msgs

We will be presented with the usual output.

.. dropdown:: ros2 pkg create output

    .. code-block:: console

        going to create a new package
        package name: python_package_that_uses_nottf2
        destination directory: ~/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: []
        creating folder ./python_package_that_uses_nottf2
        creating ./python_package_that_uses_nottf2/package.xml
        creating source folder
        creating folder ./python_package_that_uses_nottf2/python_package_that_uses_nottf2
        creating ./python_package_that_uses_nottf2/setup.py
        creating ./python_package_that_uses_nottf2/setup.cfg
        creating folder ./python_package_that_uses_nottf2/resource
        creating ./python_package_that_uses_nottf2/resource/python_package_that_uses_nottf2
        creating ./python_package_that_uses_nottf2/python_package_that_uses_nottf2/__init__.py
        creating folder ./python_package_that_uses_nottf2/test
        creating ./python_package_that_uses_nottf2/test/test_copyright.py
        creating ./python_package_that_uses_nottf2/test/test_flake8.py
        creating ./python_package_that_uses_nottf2/test/test_pep257.py

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

Package structure
-----------------

Below are the files that we will create or modify.

.. code-block:: console
    :emphasize-lines: 5, 9

    python_package_that_uses_nottf2/
    |-- package.xml
    |-- python_package_that_uses_nottf2
    |   |-- __init__.py
    |   `-- operations_showcase.py
    |-- resource
    |   `-- python_package_that_uses_nottf2
    |-- setup.cfg
    |-- setup.py
    `-- test
        |-- test_copyright.py
        |-- test_flake8.py
        `-- test_pep257.py

Add sample code for ``nottf2``
------------------------------

Create the following sample Python script. It will serve to show the operations we did earlier mathematically.

:download:`operations_showcase.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_nottf2/python_package_that_uses_nottf2/operations_showcase.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nottf2/python_package_that_uses_nottf2/operations_showcase.py
   :language: python
   :lines: 24-
   :linenos:

Update the :file:`setup.py`
---------------------------

As usual, we add the necessary entry point in :file:`setup.py`. We also add ``nottf2`` as an usual Python dependency, although,
in :program:`colcon`, this currently seems to be mostly cosmetic.

:download:`setup.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_nottf2/setup.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nottf2/setup.py
   :language: python
   :emphasize-lines: 15,24


Build and source
----------------

Before we proceed, let us build and source once.

.. include:: ../the_canonical_build_command.rst

Run Example
-----------

We run our newly created program as follows.

.. code-block:: console

    ros2 run python_package_that_uses_nottf2 operations_showcase

The result will be as follows.

.. code-block:: console

    The rotation of 3.141592653589793 radians about the x-axis is r1=geometry_msgs.msg.Quaternion(x=1.0, y=0, z=0, w=6.123233995736766e-17).
    The inverse rotation of r1 is r1_conj=geometry_msgs.msg.Quaternion(x=-1.0, y=0, z=0, w=6.123233995736766e-17).
    The rotation of 3.141592653589793 radians about the z-axis is r2=geometry_msgs.msg.Quaternion(x=0, y=0, z=1.0, w=6.123233995736766e-17).
    The quaternion multiplication of r12=r1r2 is r12=geometry_msgs.msg.Quaternion(x=6.123233995736766e-17, y=-1.0, z=6.123233995736766e-17, w=3.749399456654644e-33).

Note that the results are reasonably close to the ones we calculated mathematically. However, given the limitations
on current computers related to floating point accuracy, you can **always** expect a level of inaccuracy. This is **not**
limited or affected by the use of quaternions, this is an inherent limitation of our computers.