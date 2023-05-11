Creating a Python Library (for :program:`ament_python`)
====================================================

Let us start, as already recommended in this tutorial, with a template by :program:`ros2 pkg create`.

.. code:: bash

   cd ~/ros2_tutorial_workspace
   ros2 pkg create python_package_with_a_library --build-type ament_python --library-name sample_python_library
   
which outputs the forever beautiful wall of text we're by now used to, with a minor difference regarding the additional library template, as highlighted below.

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

The folders, Mason, what do they mean?
--------------------------------------

The ROS2 package created from the template has a structure like so. In particular, we can see that :file:`python_package_with_a_library` is repeated twice in a row. This is a common source of error, so don't forget!

.. code-block:: console
   :emphasize-lines: 1,2
   
   python_package_with_a_library
      └── python_package_with_a_library
         └── sample_python_library
            __init__.py
         __init__.py
      └── resource
      └── test
      
Whenever there is a :file:`__init__.py`, that folder is understood by Python as a `module <https://docs.python.org/3.10/tutorial/modules.html>`_. Hence, to help clarify, our project structure with a few comments means

.. code-block:: console
   
   python_package_with_a_library --> The ROS2 package, depends on (1), (2), and (3)  
      └── python_package_with_a_library --> The Python module, inside the ROS2 package, with the same name.
         └── sample_python_library --> A sample library inside the module.
            __init__.py --> Makes Python treat this folder as a module
         __init__.py --> Makes Python treat this folder as a module
      └── resource
         python_package_with_a_library --> (3) The setup.py installs this file so that ROS2 knows that your package exists.
      └── test --> Sample test files
      package.xml --> (1) The first file colcon looks into, to know what to do with this package.
      setup.cfg --> Used by setup.py 
      setup.py --> (2) The second file colcon goes through, running the setup instructions

- `Resource file <https://answers.ros.org/question/367328/ament_python-package-doesnt-explicitly-install-a-marker-in-the-package-index/>`_
- `setup.cfg <https://docs.python.org/3.10/distutils/configfile.html>`_

Overview of the library
-----------------------

.. note::

   If you have created the bad habit of declaring all/too many things in your :file:`__init__.py` file, take the hint and start breaking the definitions into different files and use the :file:`__init__.py` just to export the relevant parts of your library. 

For the sake of the example, let us create a library with a Python :code:`function` and another one with a :code:`class`. To guide our next steps, we first draw a quick overview of what our :code:`python_package_with_a_library` will look like.

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
      
With respect to the highlighted files, we will

#. Create the :code:`_sample_function.py`.
#. Create the :code:`_sample_class.py`.
#. Modify :file:`__init__.py` to use the new function and class.      
      
All other files and directories will remain as-is, in the way they were generated by :program:`ros2 pkg create`.      
      
Create the sample function
--------------------------

Add a new file to :file:`python_package_with_a_library/python_package_with_a_library/sample_python_library` called :file:`_sample_function.py` with the following content.

:download:`_sample_function.py <../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_function.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_function.py
   :language: python
   :linenos:
   :lines: 26-

The function has two parameters, :code:`a` and :code:`b`. For simplicity, we're expecting arguments of type :code:`float` and returning a :code:`float`, but it could be any Python function.

Create the sample class
-----------------------

Add a new file to :file:`python_package_with_a_library/python_package_with_a_library/sample_python_library` called :file:`_sample_class.py` with the following content.

:download:`_sample_class.py <../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_class.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_class.py
   :language: python
   :linenos:
   :lines: 26-
   
The class is quite simple with a `private data member <https://docs.python.org/3/tutorial/classes.html#private-variables>`_ and a method to retrieve it.
   
Modify the :code:`__init__.py` to export the symbols
----------------------------------------------------

With the necessary files created and properly organized, the last step is to :code:`import` the function and the class. We modify the :file:`python_package_with_a_library/python_package_with_a_library/sample_python_library/__init__.py` to have the following contents.

:download:`__init__.py <../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/__init__.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/__init__.py
   :language: python
   :linenos:
   :lines: 24-

Build and source
----------------

No surprise here, right?

.. include:: the_canonical_build_command.rst

If it builds without any unexpected issues, we're good to go!
