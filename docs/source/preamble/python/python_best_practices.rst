Python Best Practices
=====================

.. warning::
   This tutorial expects prior knowledge in Python and objected-oriented programming.
   As such, this section is not meant to be a comprehensive Python tutorial. You have better resources
   made by smarter people available online, e.g. `The Python Tutorial <https://docs.python.org/3.10/tutorial/index.html>`_.

.. _Python Terminology:

Terminology
-----------

Let's go through the Python terminology used in this tutorial. This terminology is not necessarily uniform with other
sources/tutorials you might find elsewhere. It is based on my interpretation of
`The Python Tutorial on Modules <https://docs.python.org/3.10/tutorial/modules.html>`_.

=======  ====================================================================================================
Term     Definition
script   A file that can be executed, in contrast with a file with only definitions (i.e. a library). In practice any file meant to be executed.
module   A file with content that is meant to be imported by other modules and scripts. In practice a folder with an :file:`__init__.py`.
package  A collection of modules. In practice, something with a :code:`setup.py`.
=======  ====================================================================================================

The difference between *scripts* and *modules*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

According to `The Python Tutorial on Modules <https://docs.python.org/3.10/tutorial/modules.html>`_, the definition of
*script* and *module* is not disjoint, in fact, it is said that

  *[...] you can make the file usable as a script as well as an importable module [...]*

In the official documentation, a Python script is defined as

  *[...] a [script is a] somewhat longer program, [for when] you are better off using a text editor to prepare the input for the
  interpreter and running it with [a script] as input instead [of using an interactive instance of the interpreter].*

and a module is defined as

  *[A module is a file] to put definitions [...] and use them in a script or in an interactive instance of the interpreter.*

There are more profound differences in how the Python interpreter handles *scripts* and *modules*, but in the wild the
the difference is usually as I described in :ref:`Python Terminology`.

The difference between *modules* and *packages*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

According to `Holy Book of Modules <https://docs.python.org/3.10/tutorial/modules.html>`_, a definition of packages is
given *en passant* as follows

   *Suppose you want to design a collection of modules (a “package”) [...]*

In practice, the line between modules and packages tends to be somewhat blurred. It could be a single folder with many modules but at the same time `they <https://docs.python.org/3.10/tutorial/modules.html>`_
come up with namings such as submodule

   *Packages are a way of structuring Python’s module namespace [...]. For example, the module name A.B designates a submodule named B in a package named A.*

What most people want to say when they mention a package is, usually, something with a :code:`setup.py` that can be built into a :code:`wheel` or something similar. We are not going to see
that in this preamble, but we will see a lot in the ROS2 tutorial.

Use a :code:`venv`
------------------

We already know that it is a good practice to :ref:`Isolate your environment with a venv`. So, let's turn that into a reflex
and do so for this whole section.

.. code-block:: console

   cd ~
   source ros2tutorial_venv/bin/activate

Minimalist module: something to start with
------------------------------------------

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 2
      
      python/
        └── minimalist_module.py

Let's start with a minimalist module (in this case also a script) that prints a string periodically,
as follows. Create a file in :file:`~/ros2_tutorials_preamble/python` called :file:`minimalist_module.py` with the following
contents.

:download:`minimalist_module.py <../../../../preamble/python/minimalist_module.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_module.py
   :language: python
   :linenos:
   :lines: 1-

Running a Python script on the terminal
---------------------------------------

There are a few ways to run this module in the command line. Without worrying about file permissions, specifying that
the file must be interpreted by Python (and which version of Python) is the most general way to run a script

.. code-block:: commandline

   cd ~/ros2_tutorials_preamble/python
   python3 minimalist_module.py

which will output

.. hint::

   You can end the :program:`minimalist_module.py` by pressing :kbd:`CTRL+C` in the terminal in which it is running.

.. code-block:: commandline

   Howdy!
   Howdy!
   Howdy!

Another way to run a Python script is to execute it directly in the terminal. This can be done with

.. code-block:: commandline

   cd ~/ros2_tutorials_preamble/python
   ./minimalist_module.py

which will result in

.. code-block:: commandline

   bash: ./minimalist_module.py: Permission denied

because our file does not have the permission to run as an executable. To give it that permission, we must run **ONCE**

.. code-block::

   cd ~/ros2_tutorials_preamble/python
   chmod +x minimalist_module.py

and now we can run it properly with

.. code-block:: commandline

   cd ~/ros2_tutorials_preamble/python
   ./minimalist_module.py

resulting in

.. code-block:: commandline

   Howdy!
   Howdy!
   Howdy!

Note that for this second execution strategy to work, we **MUST** have the `#!` at the beginning of the first line, called `shebang <https://en.wikipedia.org/wiki/Shebang_(Unix)>`_,
which specifies what program will be used to interpret that file. In general, Ubuntu does not guess the file type by the extension
when running it. 

.. literalinclude:: ../../../../preamble/python/minimalist_module.py
   :language: python
   :lines: 1

If we remove the shebang and try to execute the script, it will return the following errors, because Ubuntu doesn't know
what to do with that file.

.. code-block::

   ./minimalist_module.py: line 2: import: command not found
   ./minimalist_module.py: line 5: syntax error near unexpected token `('
   ./minimalist_module.py: line 5: `def main() -> None:'


It's dangerous to go alone: Always wrap the :code:`main` function on a `try--except` block
------------------------------------------------------------------------------------------

When a module is run directly, its :code:`__name__` property will be :code:`'__main__'`.

It is good practice to wrap the :code:`main()` call in a :code:`try--except` block
with at least the :code:`KeyboardInterrupt` clause. This allows the user to shutdown
the module cleanly either through the terminal or through :program:`PyCharm`. We have done so in the example as follows

.. literalinclude:: ../../../../preamble/python/minimalist_module.py
   :language: python
   :lines: 12-

This is of particular importance when hardware is used, otherwise, the connection with it might be left in an undefined
state causing difficult-to-understand problems at best and physical harm at worst.

The :code:`Exception` clause in our example is very broad, but a **MUST** in code that is still under development.
Exceptions of all sorts can be generated when there is a communication error with the hardware, software (internet, etc),
or other issues.

This broad :code:`Exception` clause could be replaced for a less broad exception handling if that makes sense in a given
application, but that is usually not necessary nor safe. When handling hardware, it is, in general, **IMPOSSIBLE** to
test the code of all combinations of inputs and states. As `they say <https://darkestdungeon.fandom.com/wiki/Narrator_(Darkest_Dungeon)>`_,

   *Be wary, for overconfidence is a slow and insidious [source for terrible bugs and failed demos]*

.. hint::

   Catching all :code:`Exceptions` might make debugging more difficult in some cases. At your own risk, you can remove
   this clause temporarily when trying to debug a stubborn bug, at the risk of forgetting to put it back and ruining
   your hardware.

.. _Python package:

Minimalist Package: Use packages to organize your code
------------------------------------------------------

.. admonition:: In this step, we'll work on these.

   .. code-block:: console
      :emphasize-lines: 3,4
      
      python/
        └── minimalist_module.py
        └── minimalist_package/
              └── __init__.py

A Python package is a folder that has an :file:`__init__.py`. Yes, a :file:`__init__.py` can even be empty and it would
still be considered a Python package.

Anyways, back to the example. First, let's make a folder for our package

.. code-block::

   cd ~/ros2_tutorials_preamble/python
   mkdir minimalist_package

then, let's create a file :file:`__init__.py` in :file:`~/ros2_tutorials_preamble/python/minimalist_package` with the
following contents

:download:`__init__.py <../../../../preamble/python/minimalist_package/__init__.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_package/__init__.py
   :language: python
   :linenos:
   :lines: 1-

When adding imports to the :file:`__init__.py`, the folder that we use to open in Pycharm and that we call to execute
the scripts is *extremely* relevant. When packages are deployed (e.g. in `PyPI <https://pypi.org/>`_ or ROS2), the "correct"
way to import in :file:`__init__.py` is to use :code:`import <PACKAGE_NAME>.<THING_TO_IMPORT>`, which is why we're doing
it this way.

.. note::

   Relative imports such as :code:`.<THING_TO_IMPORT>` might work in some cases, and that is fine. It is a supported
   and valid way to import. However, don't be surprised when it doesn't work in ROS2, PyPI packages, etc, and generates 
   a lot of frustration.

Minimalist class: Use classes profusely
---------------------------------------

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 5
      
      python/
        └── minimalist_module.py
        └── minimalist_package/
              └── __init__.py
              └── _minimalist_class.py

As you are familiar with object-oriented programing, you know that classes are central to this paradigm.
As a memory refresher, let's make a class that honestly does nothing really useful but illustrates all
the basic points in a Python class.

Create a file in :file:`~/ros2_tutorials_preamble/python/minimalist_package` called :file:`_minimalist_class.py` with the following
contents.

:download:`_minimalist_class.py <../../../../preamble/python/minimalist_package/_minimalist_class.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_package/_minimalist_class.py
   :language: python
   :linenos:
   :lines: 31-

Use type hints profusely
------------------------

.. note::

   For more info, check out the documentation on `Python typing <https://docs.python.org/3.10/library/typing.html>`_ and the
   `type hints cheat sheet <https://mypy.readthedocs.io/en/stable/cheat_sheet_py3.html>`_

Before you flood my inbox with complaints, let me vent for you. A *preemptive* vent.

  *But, you know, one of the cool things in Python is that we don't have to explicitly type variables. Do you want to turn Python into C?? Why do you love C++ so much you unpythonic Python hater????*

The dynamic typing nature of Python is, no doubt, a strong point of the language. Note that adding type hints does not impede your code to be used with other types as arguments. Type hints are, to no one's surprise, hints to let users (and some automated tools) know what types your functions were made for, e.g. to allow your favorite :abbr:`IDE (Integrated Development Environment)` to help you with code suggestions.

In these tutorials, we are not going to use any complex form of type hints. We're basically going to attain ourselves to the simplest two forms, the (attribute, argument, etc) type, and the return types.

For attributes we use :code:`<attribute>: type`, as shown below

.. literalinclude:: ../../../../preamble/python/minimalist_package/_minimalist_class.py
   :language: python
   :lines: 37

For method arguments we use :code:`<argument>: type` and for return types we use :code:`def method(args) -> return type`, as shown below in our example

.. literalinclude:: ../../../../preamble/python/minimalist_package/_minimalist_class.py
   :language: python
   :lines: 62-64
   :emphasize-lines: 1

Unit tests: always test your code
---------------------------------

.. note::

   For a comprehensive tutorial on unit testing go through the `unittest docs <https://docs.python.org/3.10/library/unittest.html>`_.

.. admonition:: In this step, we'll work on these.

   .. code-block:: console
      :emphasize-lines: 6,7
      
      python/
        └── minimalist_module.py
        └── minimalist_package/
              └── __init__.py
              └── _minimalist_class.py
        └── tests/
              └── test_minimalist_class.py

`Unit testing <https://en.wikipedia.org/wiki/Unit_testing>`_ is a flag that has been waved by programming enthusiasts 
and is often a good measurement of code maturity.

The elephant in the room is that writing unit tests is **boring**. Yes, we know, *very* boring.

Unit tests are boring because they are an *investment*. Unit testing won't necessarily make your
code `[...] better, faster, [...] <https://www.youtube.com/watch?v=gAjR4_CbPpQ>`_ *right now*. However, without tests, don't 
be surprised after some point if your implementations make you drown in `tech debt <https://en.wikipedia.org/wiki/Technical_debt>`_.
Dedicating a couple of minutes now to make a couple of tests when your codebase is still in its infancy makes it more manageable 
and less boresome.

Back to the example, a good practice is to create a folder name :file:`tests` at the same level as the packages to be tested, like so

.. code-block::

   cd ~/ros2_tutorials_preamble/python
   mkdir tests

Then, we create a file named :file:`test_minimalist_class.py` with the contents below in the :file:`tests` folder.

.. note::
 
   The prefix :file:`test_` is important as it is used by some frameworks to automatically discover tests. So it is better not to use
   that prefix if that file does not contain a unit test.

:download:`test_minimalist_class.py <../../../../preamble/python/tests/test_minimalist_class.py>`

.. literalinclude:: ../../../../preamble/python/tests/test_minimalist_class.py
   :language: python
   :linenos:
   :lines: 6-

Running the tests
^^^^^^^^^^^^^^^^^

For a little jolt of instant gratification, let's run the tests before we proceed with the explanation.

There are many ways to run tests written with :code:`unittest`. What we'll do that works with this example
is

.. code-block:: commandLine

   cd ~/ros2_tutorials_preamble/python
   python3 -m unittest tests/test_minimalist_class.py

which will output

.. code-block:: commandLine
   
   ============================= test session starts ==============================
   collecting ... collected 5 items
   
   tests/test_minimalist_class.py::TestMinimalistClass::test_attribute PASSED [ 20%]
   tests/test_minimalist_class.py::TestMinimalistClass::test_get_set_private_attribute PASSED [ 40%]
   tests/test_minimalist_class.py::TestMinimalistClass::test_method PASSED  [ 60%]
   tests/test_minimalist_class.py::TestMinimalistClass::test_private_attribute PASSED [ 80%]
   tests/test_minimalist_class.py::TestMinimalistClass::test_static_method PASSED [100%]
   
   ============================== 5 passed in 0.03s ===============================
   
   Process finished with exit code 0

Yay! We've done it!

Always use :code:`unittest`
^^^^^^^^^^^^^^^^^^^^^^^^^^^

There are many test frameworks for Python. Nonetheless, the `unittest module <https://docs.python.org/3.10/library/unittest.html>`_
is built into Python so, unless you have a very good reason not to use it, `just [use] it <https://www.youtube.com/watch?v=ZXsQAXx_ao0>`_.

We import the :code:`unittest` module along with the class that we want to test, namely :code:`MinimalistClass`.

.. literalinclude:: ../../../../preamble/python/tests/test_minimalist_class.py
   :language: python
   :lines: 6-7

Test them all
^^^^^^^^^^^^^

.. note::

   Good unit tests will not only let you know when something broke but also *where* it broke. A failed test of a high-level function might not give
   you too much information, whereas a failed test of a lower-level (more fundamental) function will allow you to pinpoint the issue.

Unit tests are somewhat like insurance. The more coverage you have, the better. In this example, we test all the 
elements in the class. Each test will be based on one or more asserts. For more info check the `unittest docs <https://docs.python.org/3.10/library/unittest.html>`_.

In a few words, we make a subclass of :code:`unittest.TestCase` and create methods within it that test one part
of the code, hence the name unit tests. 

.. literalinclude:: ../../../../preamble/python/tests/test_minimalist_class.py
   :language: python
   :lines: 10-31

If one of the :code:`asserts` fails, then the related test will fail, and the test framework will let us know which one.

The main function
^^^^^^^^^^^^^^^^^

Generally, a test script based on `unittest` will have the following main function. It will run all available tests in our test class.
For more info and alternatives check the `unittest docs <https://docs.python.org/3.10/library/unittest.html>`_.

.. literalinclude:: ../../../../preamble/python/tests/test_minimalist_class.py
   :language: python
   :lines: 34-35
   :emphasize-lines: 2
