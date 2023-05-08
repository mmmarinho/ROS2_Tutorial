Creating a Python Library (for :code:`ament_python`)
====================================================

Let us start, as already recommended in this tutorial, with a template by :program:`ros2 pkg create`.

.. code:: bash

   cd ~/ros2_tutorial_workspace
   ros2 pkg create python_package_with_a_library --build-type ament_python --library-name sample_python_library
   
which outputs the forever beautiful wall of text we're by now used to, with a minor difference regarding the additional library template

.. code-block:: console
   :emphasize-lines: 25, 26

   going to create a new package
   package name: python_package_with_a_library
   destination directory: /home/murilo/git/ROS2_Tutorial/ros2_tutorial_workspace/src
   package format: 3
   version: 0.0.0
   description: TODO: Package description
   maintainer: ['murilo <murilomarinho@ieee.org>']
   licenses: ['TODO: License declaration']
   build type: ament_python
   dependencies: []
   library_name: sample_python_library
   creating folder ./python_package_with_a_library
   creating ./python_package_with_a_library/package.xml
   creating source folder
   creating folder ./python_package_with_a_library/python_package_with_a_library
   creating ./python_package_with_a_library/setup.py
   creating ./python_package_with_a_library/setup.cfg
   creating folder ./python_package_with_a_library/resource
   creating ./python_package_with_a_library/resource/python_package_with_a_library
   creating ./python_package_with_a_library/python_package_with_a_library/__init__.py
   creating folder ./python_package_with_a_library/test
   creating ./python_package_with_a_library/test/test_copyright.py
   creating ./python_package_with_a_library/test/test_flake8.py
   creating ./python_package_with_a_library/test/test_pep257.py
   creating folder ./python_package_with_a_library/python_package_with_a_library/sample_python_library
   creating ./python_package_with_a_library/python_package_with_a_library/sample_python_library/__init__.py

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


For the sake of the example, let us create a library with a Python :code:`function` and another one with a :code:`class`.

Overview of the library
-----------------------

.. note::

   If you have created the bad habit of declaring all/too many things in your :file:`__init__.py` file, take the hint and start breaking the definitions into different files and use the :file:`__init__.py` just to export the relevant parts of your library. 

To guide our next steps, we first draw a quick overview of what our :code:`python_package_with_a_library` will look like.
With respect to the highlighted files, we will

#. Create the :code:`_sample_function.py`.
#. Create the :code:`_sample_class.py`.
#. Modify :file:`__init__.py` to use the new :code:`function` and :code:`class`.

.. code-block:: console
   :emphasize-lines: 4,5,6
   
   python_package_with_a_library
      └── python_package_with_a_library
         └── sample_python_library
            __init__.py
            _sample_class.py
            _sample_function.py
         __init__.py
      └── resource
      └── test
      
Create the sample :code:`function`
----------------------------------

Add a new file to :file:`python_package_with_a_library/python_package_with_a_library/sample_python_library` called :file:`_sample_function.py` with the following content.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_function.py
   :language: python
   :linenos:
   :lines: 26-
      
Create the sample :code:`class`
-------------------------------

Add a new file to :file:`python_package_with_a_library/python_package_with_a_library/sample_python_library` called :file:`_sample_class.py` with the following content.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_class.py
   :language: python
   :linenos:
   :lines: 26-
   
Modify the :code:`__init__.py` to export the symbols
----------------------------------------------------

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/__init__.py
   :language: python
   :linenos:
   :lines: 24-
