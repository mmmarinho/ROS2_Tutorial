Using a Python Library from another package (for :program:`ament_python`)
======================================================================

Let us create a package with a Node that uses the library we created in the prior example. 

Note that we must add the :code:`python_package_with_a_library` as a dependency to our new package. The easiest way to
do so is through :program:`ros2 pkg create`. We also add :code:`rclpy` as a dependency so that our Node can do something useful.


.. code :: console

   cd ~/ros2_tutorial_workspace/src
   ros2 pkg create python_package_that_uses_the_library \
   --dependencies rclpy python_package_with_a_library \
   --build-type ament_python \
   --node-name node_that_uses_the_library
   
resulting in yet another version of our favorite wall of text

.. code-block:: console
   :emphasize-lines: 10

   going to create a new package
   package name: python_package_that_uses_the_library
   destination directory: /root/ros2_tutorial_workspace/src
   package format: 3
   version: 0.0.0
   description: TODO: Package description
   maintainer: ['root <murilo.marinho@manchester.ac.uk>']
   licenses: ['TODO: License declaration']
   build type: ament_python
   dependencies: ['rclpy', 'python_package_with_a_library']
   node_name: node_that_uses_the_library
   creating folder ./python_package_that_uses_the_library
   creating ./python_package_that_uses_the_library/package.xml
   creating source folder
   creating folder ./python_package_that_uses_the_library/python_package_that_uses_the_library
   creating ./python_package_that_uses_the_library/setup.py
   creating ./python_package_that_uses_the_library/setup.cfg
   creating folder ./python_package_that_uses_the_library/resource
   creating ./python_package_that_uses_the_library/resource/python_package_that_uses_the_library
   creating ./python_package_that_uses_the_library/python_package_that_uses_the_library/__init__.py
   creating folder ./python_package_that_uses_the_library/test
   creating ./python_package_that_uses_the_library/test/test_copyright.py
   creating ./python_package_that_uses_the_library/test/test_flake8.py
   creating ./python_package_that_uses_the_library/test/test_pep257.py
   creating ./python_package_that_uses_the_library/python_package_that_uses_the_library/node_that_uses_the_library.py

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

We'll be working on the following file structure.

.. code-block:: console
    :emphasize-lines: 5

    python_package_that_uses_the_library/
    |-- package.xml
    |-- python_package_that_uses_the_library
    |   |-- __init__.py
    |   `-- node_that_uses_the_library.py
    |-- resource
    |   `-- python_package_that_uses_the_library
    |-- setup.cfg
    |-- setup.py
    `-- test
        |-- test_copyright.py
        |-- test_flake8.py
        `-- test_pep257.py

The sample Node
---------------

Given that it was created from a template, the file :file:`node_that_uses_the_library.py` is currently *mostly* empty. Let us replace its contents with

:download:`node_that_uses_the_library.py <../../ros2_tutorial_workspace/src/python_package_that_uses_the_library/python_package_that_uses_the_library/node_that_uses_the_library.py>`

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_library/python_package_that_uses_the_library/node_that_uses_the_library.py
   :language: python
   :linenos:
   :lines: 24-

Indeed, the most difficult part is to make and configure the library itself. After that, to use it in another package, it is straightforward. We :code:`import` the library.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_library/python_package_that_uses_the_library/node_that_uses_the_library.py
   :language: python
   :lines: 24-29
   :emphasize-lines: 6
   
And then use the symbols we imported as we would with any other Python library.

.. literalinclude:: ../../ros2_tutorial_workspace/src/python_package_that_uses_the_library/python_package_that_uses_the_library/node_that_uses_the_library.py
   :language: python
   :lines: 40-55
   :emphasize-lines: 10, 14

Build and source
----------------

As always, this is needed so that our new package and node can be recognized by :program:`ros2 run`.

.. include:: the_canonical_build_command.rst

Run
---

.. hint::
   Remember that you can stop the node at any time with :kbd:`CTRL+C`.

.. code :: console

   ros2 run python_package_that_uses_the_library node_that_uses_the_library
   
Which outputs something similar to the shown below, but with different numbers and strings as they are randomized.

.. code :: console

   [INFO] [1753585839.509922172] [node_that_uses_the_library]: sample_function_for_square_of_sum(0.9787232004970391,1.7320908702316369) returned 7.348512926060575.
   [INFO] [1753585839.510400755] [node_that_uses_the_library]: sample_class_with_random_name.get_name() returned GQkUZgSkje.
   [INFO] [1753585839.993999505] [node_that_uses_the_library]: sample_function_for_square_of_sum(0.7637556876478347,1.0377535114838756) returned 3.2454353945561767.
   [INFO] [1753585839.994351922] [node_that_uses_the_library]: sample_class_with_random_name.get_name() returned EztAWYuFMB.
   [INFO] [1753585840.494895006] [node_that_uses_the_library]: sample_function_for_square_of_sum(0.26638758438777976,1.1809445792770386) returned 2.0947703919786846.
   [INFO] [1753585840.495950422] [node_that_uses_the_library]: sample_class_with_random_name.get_name() returned ITLIHPOMgv.
   [INFO] [1753585840.994468589] [node_that_uses_the_library]: sample_function_for_square_of_sum(0.5244531764161572,1.7524376840394509) returned 5.184231990426279.
   [INFO] [1753585840.994695797] [node_that_uses_the_library]: sample_class_with_random_name.get_name() returned LGtybBngKv.
    
