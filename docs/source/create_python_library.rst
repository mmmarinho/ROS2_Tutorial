Creating a Python Library (for :program:`ament_python`)
=======================================================

Let us start, as already recommended in this tutorial, with a template by :program:`ros2 pkg create`.

.. code :: console

   cd ~/ros2_tutorial_workspace/src
   ros2 pkg create python_package_with_a_library \
   --build-type ament_python \
   --library-name sample_python_library
   
which outputs the forever beautiful wall of text we're now used to, with a minor difference regarding the additional library template, as highlighted below.

.. code-block:: console
   :emphasize-lines: 25, 26

   going to create a new package
   package name: python_package_with_a_library
   destination directory: /root/ros2_tutorial_workspace/src
   package format: 3
   version: 0.0.0
   description: TODO: Package description
   maintainer: ['root <murilo.marinho@manchester.ac.uk>']
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

The folders/files, Mason, what do they mean?
--------------------------------------------

The ROS2 package created from the template has a structure like so. In particular, we can see that :file:`python_package_with_a_library` is repeated twice in a row. This is a common source of error, so don't forget!
The first is the name of the :program:`ROS2` package, and the second is the name of Python package that will be installed by :program:`ROS2`.

.. code-block:: console
   :emphasize-lines: 1,3
   
   python_package_with_a_library/
   |-- package.xml
   |-- python_package_with_a_library
   |   |-- __init__.py
   |   `-- sample_python_library
   |       `-- __init__.py
   |-- resource
   |   `-- python_package_with_a_library
   |-- setup.cfg
   |-- setup.py
   `-- test
       |-- test_copyright.py
       |-- test_flake8.py
       `-- test_pep257.py
      
We learned the meaning of most of those in the preamble, namely :ref:`Python Best Practices`. To quickly clarify a few things, see the table below.

.. list-table:: ROS2 Python package folders/files explained
   :header-rows: 1

   * - File/Directory
     - Meaning
   * - :file:`python_package_with_a_library`
     - The ROS2 package folder.
   * - :file:`python_package_with_a_library/python_package_with_a_library`
     - The Python package, as we saw in the preamble.
   * - :file:`sample_python_library`
     - The module corresponding to our sample library.
   * - :file:`resource/python_package_with_a_library`
     - A file for ROS2 to index this package correctly. See `Resource file <https://answers.ros.org/question/367328/ament_python-package-doesnt-explicitly-install-a-marker-in-the-package-index/>`_.
   * - :file:`test`
     - The folder containing the tests, as we already saw in the preamble.
   * - :file:`setup.cfg`
     - Used by setup.py, see `setup.cfg docs <https://docs.python.org/3.10/distutils/configfile.html>`_.
   * - :file:`setup.py`
     - The instructions to make the package installable, as we saw in the preamble.

Overview of the library
-----------------------

.. hint::
   If you have created the bad habit of declaring all/too many things in your :file:`__init__.py` file, take the hint and start breaking the definitions into different files and use the :file:`__init__.py` just to export the relevant parts of your library. 

For the sake of the example, let us create a library with a Python :code:`function` and another one with a :code:`class`. To guide our next steps, we first draw a quick overview of what our :code:`python_package_with_a_library` will look like.

.. code-block:: console
   :emphasize-lines: 6-8
   
   python_package_with_a_library/
   |-- package.xml
   |-- python_package_with_a_library
   |   |-- __init__.py
   |   `-- sample_python_library
   |       |-- __init__.py
   |       |-- _sample_class.py
   |       `-- _sample_function.py
   |-- resource
   |   `-- python_package_with_a_library
   |-- setup.cfg
   |-- setup.py
   `-- test
       |-- test_copyright.py
       |-- test_flake8.py
       `-- test_pep257.py
      
With respect to the highlighted files, we will

#. Create the :code:`_sample_function.py`.
#. Create the :code:`_sample_class.py`.
#. Modify :file:`__init__.py` to use the new function and class.      
      
All other files and directories will remain as-is, in the way they were generated by :program:`ros2 pkg create`.      
      
Create the sample function
--------------------------

Create a new file with the following contents and name.

:download:`~/ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_function.py <../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_function.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_function.py
   :language: python
   :linenos:
   :lines: 26-

The function has two parameters, :code:`a` and :code:`b`. For simplicity, we're expecting arguments of type :code:`float` and returning a :code:`float`, but it could be any Python function.

Create the sample class
-----------------------

Create a new file with the following contents and name. 

:download:`~/ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_class.py <../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_class.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/_sample_class.py
   :language: python
   :linenos:
   :lines: 26-
   
The class is quite simple with a `private data member <https://docs.python.org/3/tutorial/classes.html#private-variables>`_ and a method to retrieve it.

Modify the :file:`__init__.py`
------------------------------

:download:`~/ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/__init__.py <../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/__init__.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_with_a_library/python_package_with_a_library/sample_python_library/__init__.py
   :language: python
   :linenos:
   :lines: 24-

The :file:`__init__.py` file should import from the internal modules to expose their contents to other packages.

The absolute path in terms of :program:`ROS2` is needed to guarantee this will work well. It is clever to stay away from
`package relative imports <https://docs.python.org/3/reference/import.html#package-relative-imports>`_ in this case.

Build and source
----------------

No surprise here, right?

.. include:: the_canonical_build_command.rst

If it builds without any unexpected issues, we're good to go!
