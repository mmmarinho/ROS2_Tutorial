Creating a Python package (for :program:`ament_python`)
=======================================================

.. note::
   This is **NOT** the only way to build Python packages in ROS2.

Packages in ROS2 can either rely on :program:`CMake` or directly use setup tools available in Python. 
For pure Python projects, it might be easier to use :program:`ament_python`, so we start this tutorial with it.

Let us build the simplest of Python packages and start from there.

.. code :: console

   cd ~/ros2_tutorial_workspace/src
   ros2 pkg create --build-type ament_python the_simplest_python_package
   
which will result in the output below, meaning the package has been generated successfully.

..  code :: console
    
    going to create a new package
    package name: the_simplest_python_package
    destination directory: /home/murilo/ros2_tutorial_workspace/src
    package format: 3
    version: 0.0.0
    description: TODO: Package description
    maintainer: ['murilo <murilomarinho@ieee.org>']
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
    It is recommended to use one of the ament license identitifers:
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
    --- stderr: the_simplest_python_package                   
    /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
      warnings.warn(
    ---
    Finished <<< the_simplest_python_package [1.72s]

    Summary: 1 package finished [1.89s]
      1 package had stderr output: the_simplest_python_package

meaning that :program:`colcon` succesfully built the example package. Sadly, in this version of ROS2, all :program:`ament_python` packages will output a :code:`SetuptoolsDeprecationWarning`.
This is related to `this issue on Github <https://github.com/colcon/colcon-core/issues/454#issuecomment-1262592774>`_. Until that is fixed, just ignore it.






