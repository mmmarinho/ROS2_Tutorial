Creating a Python package (for :program:`ament_python`)
=======================================================

.. note::
   This is **NOT** the only way to build Python packages in ROS2.

Packages in ROS2 can either rely on :program:`CMake` or directly use setup tools available in Python. 
For pure Python projects, it might be easier to use :program:`ament_python`, so we start this tutorial with it.

Let us build the simplest of Python packages and start from there.

.. code-block:: console

   cd ~/ros2_tutorial_workspace/src
   ros2 pkg create the_simplest_python_package \
   --build-type ament_python

.. hint::

   If you don't explicitly define the maintainer name and email, :program:`ros2 pkg create` will try to:

   #. Define the maintainer's name as the currently logged-in user's name (see `source <https://github.com/ros2/ros2cli/blob/cf43e92fb17b5e51c95406f01fa63aeb65adf75f/ros2pkg/ros2pkg/verb/create.py#L82>`_ and `source <https://docs.python.org/3/library/getpass.html#getpass.getuser>`_).
   #. Define the maintainer's email by getting it from :program:`git` (see `source <https://github.com/ros2/ros2cli/blob/cf43e92fb17b5e51c95406f01fa63aeb65adf75f/ros2pkg/ros2pkg/verb/create.py#L109>`_). It will get whatever is defined with :program:`git config --global user.email`.

which will result in the output below, meaning the package has been generated successfully.

..  code-block:: console
    :emphasize-lines: 7
    
    going to create a new package
    package name: the_simplest_python_package
    destination directory: /root/ros2_tutorial_workspace/src
    package format: 3
    version: 0.0.0
    description: TODO: Package description
    maintainer: ['root <murilo.marinho@manchester.ac.uk>']
    licenses: ['TODO: License declaration']
    build type: ament_python
    dependencies: []
    creating folder ./the_simplest_python_package
    creating ./the_simplest_python_package/package.xml
    creating source folder
    creating folder ./the_simplest_python_package/the_simplest_python_package
    creating ./the_simplest_python_package/setup.py
    creating ./the_simplest_python_package/setup.cfg
    creating folder ./the_simplest_python_package/resource
    creating ./the_simplest_python_package/resource/the_simplest_python_package
    creating ./the_simplest_python_package/the_simplest_python_package/__init__.py
    creating folder ./the_simplest_python_package/test
    creating ./the_simplest_python_package/test/test_copyright.py
    creating ./the_simplest_python_package/test/test_flake8.py
    creating ./the_simplest_python_package/test/test_pep257.py

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

We can build the workspace that now has this empty package using :program:`colcon`

.. code :: console

   cd ~/ros2_tutorial_workspace
   colcon build
  
which will now output

.. code :: console

    Starting >>> the_simplest_python_package
    Finished <<< the_simplest_python_package [0.49s]

    Summary: 1 package finished [0.55s]

meaning that :program:`colcon` successfully built the example package. It is important to note that :program:`ROS2` does not know this package exists yet.
