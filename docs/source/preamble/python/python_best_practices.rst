.. include:: ../../the_section_is_optional.rst

.. _Python Best Practices:

(Murilo's) Python Best Practices
================================

.. warning::
   This tutorial expects prior knowledge in Python and objected-oriented programming.
   As such, this section is not meant to be a comprehensive Python tutorial. You have better resources
   made by smarter people available online, e.g. `The Python Tutorial <https://docs.python.org/3.10/tutorial/index.html>`_.

.. _Python Terminology:

Terminology
-----------

Let's go through the Python terminology used in this tutorial. This terminology is not necessarily uniform with other
sources/tutorials you might find elsewhere. It is based on my interpretation of
`The Python Tutorial on Modules <https://docs.python.org/3.10/tutorial/modules.html>`_, the `Python Glossary <https://docs.python.org/3.10/glossary.html>`_, and my own experience.

.. list-table:: (Murilo's) Python Glossary
   :header-rows: 1

   * - Term
     - Book Definition
     - Use in the wild
   * - script
     - A Python file that can be executed.
     - Any Python file *meant to be* executed.
   * - module
     - A file with content that is meant to be imported by other modules and scripts.
     - This term is used very loosely and can basically mean any Python file, but usually a Python file *meant to be* imported from.
   * - package
     - A collection of modules.
     - A folder with an :file:`__init__.py`, even if it doesn't have more than one module. When people say `Python Packaging <https://packaging.python.org/en/latest/>`_ it refers instead to making your package installable (e.g. with a :file:`setup.py` or :file:`pyproject.toml`), so be ready for that ambiguity.

Use a :code:`venv`
------------------

We already know that it is a good practice to :ref:`Isolate your environment with a venv`. So, let's turn that into a reflex
and do so for this whole section.

.. code-block:: console

   cd ~
   source ros2tutorial_venv/bin/activate

.. _Python package:

Minimalist package: something to start with
-------------------------------------------

.. admonition:: In this step, we'll work on these.

   .. code-block:: console
      :emphasize-lines: 2,3
      
      python/minimalist_package/
        └── minimalist_package/
              └── __init__.py


First, let's make a folder for our project

.. code-block::

   cd ~/ros2_tutorials_preamble/python/
   mkdir minimalist_package

Then, let's create a folder with the same name within it for our package. A Python package is a folder that has an :file:`__init__.py`, so for now we add an empty :file:`__init__.py` by doing so

.. code-block:: console

   cd ~/ros2_tutorials_preamble/python/minimalist_package
   mkdir minimalist_package
   cd minimalist_package
   touch __init__.py

The (empty) package is done!

.. hint::

   In :code:`PyCharm`, open the :file:`~/ros2_tutorials_preamble/python/minimalist_package` folder to correctly interact with this project.

.. warning::

   It is confusing to have two nested folders with the same name. However, this is quite common and
   starts to make sense after getting used to it (it is also the norm in ROS2). The first folder
   is supposed to be how your file system sees your package, i.e. the *project* folder,
   and the other contains the actual Python package, with the :file:`__init__.py` and other source code.

Minimalist script
-----------------

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 4
      
      python/minimalist_package/
        └── minimalist_package/
              └── __init__.py
              └── minimalist_script.py

Let's start with a minimalist script that prints a string periodically,
as follows. Create a file in :file:`~/ros2_tutorials_preamble/python/minimalist_package/minimalist_package` called :file:`minimalist_script.py` with the following
contents.

:download:`minimalist_script.py <../../../../preamble/python/minimalist_package/minimalist_package/minimalist_script.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_package/minimalist_package/minimalist_script.py
   :language: python
   :linenos:
   :lines: 1-

Running a Python script on the terminal
---------------------------------------

There are a few ways to run a script/module in the command line. Without worrying about file permissions, specifying that
the file must be interpreted by Python (and which version of Python) is the most general way to run a script

.. code-block:: console

   cd ~/ros2_tutorials_preamble/python/minimalist_package/minimalist_package
   python3 minimalist_script.py

which will output

.. hint::

   You can end the :program:`minimalist_script.py` by pressing :kbd:`CTRL+C` in the terminal in which it is running.

.. code-block:: console

   Howdy!
   Howdy!
   Howdy!

Another way to run a Python script is to execute it directly in the terminal. This can be done with

.. code-block:: console

   cd ~/ros2_tutorials_preamble/python/minimalist_package/minimalist_package
   ./minimalist_script.py

which will result in

.. code-block:: console

   bash: ./minimalist_script.py: Permission denied

because our file does not have the permission to run as an executable. To give it that permission, we must run **ONCE**

.. code-block::

   cd ~/ros2_tutorials_preamble/python/minimalist_package/minimalist_package
   chmod +x minimalist_script.py

and now we can run it properly with

.. code-block:: commandline

   cd ~/ros2_tutorials_preamble/python/minimalist_package/minimalist_package
   ./minimalist_script.py

resulting in

.. code-block:: console

   Howdy!
   Howdy!
   Howdy!

Note that for this second execution strategy to work, we **MUST** have the `#!`, called `shebang <https://en.wikipedia.org/wiki/Shebang_(Unix)>`_, at the beginning of the first line.
The path after the shebang specifies what program will be used to interpret that file. In general, differently from Windows, Ubuntu does not guess the file type by the extension
when running it. 

.. literalinclude:: ../../../../preamble/python/minimalist_package/minimalist_package/minimalist_script.py
   :language: python
   :lines: 1

If we remove the shebang line and try to execute the script, it will return the following errors, because Ubuntu doesn't know
what to do with that file.

.. code-block::

   ./minimalist_script.py: line 2: import: command not found
   ./minimalist_script.py: line 5: syntax error near unexpected token `('
   ./minimalist_script.py: line 5: `def main() -> None:'

When using :code:`if __name__=="__main__":`, just call the real :code:`main()`
------------------------------------------------------------------------------

There are multiple ways of running a Python script. In the one we just saw, the name
of the module becomes :code:`__main__`, but in others that does not happen, meaning that the :code:`if` can be completely skipped.
So, write the :code:`main()` function of a script as something standalone and, in the condition, just call it and do nothing else, as shown below

.. literalinclude:: ../../../../preamble/python/minimalist_package/minimalist_package/minimalist_script.py
   :language: python
   :lines: 17-

.. _Python try catch:

It's dangerous to go alone: Always wrap the contents of :code:`main` function on a `try--except` block
------------------------------------------------------------------------------------------------------

It is good practice to wrap the contents of :code:`main()` call in a :code:`try--except` block
with at least the :code:`KeyboardInterrupt` clause. This allows the user to shutdown
the module cleanly either through the terminal or through :program:`PyCharm`. We have done so in the example as follows

.. literalinclude:: ../../../../preamble/python/minimalist_package/minimalist_package/minimalist_script.py
   :language: python
   :lines: 5-14
   :emphasize-lines: 3,7,9

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

Minimalist class: Use classes profusely
---------------------------------------

.. admonition:: In this step, we'll work on these.

   .. code-block:: console
      :emphasize-lines: 3,5
      
      python/minimalist_package/
        └── minimalist_package/
              └── __init__.py
              └── minimalist_script.py
              └── _minimalist_class.py 

As you are familiar with object-oriented programing, you know that classes are central to this paradigm.
As a memory refresher, let's make a class that honestly does nothing really useful but illustrates all
the basic points in a Python class.

Create a file in :file:`~/ros2_tutorials_preamble/python/minimalist_package/minimalist_package` called :file:`_minimalist_class.py` with the following
contents.

:download:`_minimalist_class.py <../../../../preamble/python/minimalist_package/minimalist_package/_minimalist_class.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_package/minimalist_package/_minimalist_class.py
   :language: python
   :linenos:
   :lines: 26-

then, let's modify the :file:`__init__.py` with the
following contents

:download:`__init__.py <../../../../preamble/python/minimalist_package/minimalist_package/__init__.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_package/minimalist_package/__init__.py
   :language: python
   :linenos:
   :lines: 1-

.. note::

   When adding imports to the :file:`__init__.py`, the folder that we use to open in Pycharm and that we call to execute
   the scripts is *extremely* relevant. When packages are deployed (e.g. in `PyPI <https://pypi.org/>`_ or ROS2), the "correct"
   way to import in :file:`__init__.py` is to use :code:`import <PACKAGE_NAME>.<THING_TO_IMPORT>`, which is why we're doing
   it this way.

.. note::

   Relative imports such as :code:`.<THING_TO_IMPORT>` might work in some cases, and that is fine. It is a supported
   and valid way to import. However, don't be surprised when it doesn't work in ROS2, PyPI packages, etc, and generates 
   a lot of frustration.

Not a matter of taste: Code style
---------------------------------

It might be parsing through jibber-jabber code in l__tcode lessons with weird C-pointer logic and nested dereference operators that gets you through the door into one of those fancy companies with no dress code and free snacks, perks that `I'm totally not envious of one bit <https://www.youtube.com/watch?v=rkUkVxM6R8o>`_. In the ideal world, at least, writing easy-to-understand code with the proper style is what should keep you in that job.

So, always pay attention to the naming of classes (`PascalCase <https://en.wiktionary.org/wiki/Pascal_case>`_), files and functions (`snake_case <https://en.wikipedia.org/wiki/Snake_case>`_), etc.

Thankfully, Python has a bunch of style rules builtin the language and :abbr:`PEP (Python Enhancement Proposal)`, such as `PEP8 <https://peps.python.org/pep-0008/>`_. Take this time to read it and get inspired by `The Zen of Python <https://peps.python.org/pep-0020/>`_

.. 

   | *Beautiful is better than ugly.*
   | *Explicit is better than implicit.*
   | *Simple is better than complex.*
   | *Complex is better than complicated.*
   | *Flat is better than nested.*
   | *Sparse is better than dense.*
   | *Readability counts.*
   | *Special cases aren't special enough to break the rules.*
   | *Although practicality beats purity.*
   | *Errors should never pass silently.*
   | *Unless explicitly silenced.*
   | *In the face of ambiguity, refuse the temptation to guess.*
   | *There should be one-- and preferably only one --obvious way to do it.*
   | *Although that way may not be obvious at first* `*unless you're Dutch* <https://stackoverflow.com/questions/2470761/what-does-this-sentence-mean-in-the-zen-of-python>`_.
   | *Now is better than never.*
   | *Although never is often better than *right* now.*
   | *If the implementation is hard to explain, it's a bad idea.*
   | *If the implementation is easy to explain, it may be a good idea.*
   | *Namespaces are one honking great idea -- let's do more of those!*

.. _Type hints:

Take the (type) hint: Always use type hints
-------------------------------------------

.. note::

   For more info, check out the documentation on `Python typing <https://docs.python.org/3.10/library/typing.html>`_ and the
   `type hints cheat sheet <https://mypy.readthedocs.io/en/stable/cheat_sheet_py3.html>`_

Before you flood my inbox with complaints, let me vent for you. A *preemptive* vent.

  *But, you know, one of the cool things in Python is that we don't have to explicitly type variables. Do you want to turn Python into C?? Why do you love C++ so much you unpythonic Python hater????*

The dynamic typing nature of Python is, no doubt, a strong point of the language. Note that adding type hints does not impede your code to be used with other types as arguments. Type hints are, to no one's surprise, hints to let users (and some automated tools) know what types your functions were made for, e.g. to allow your favorite :abbr:`IDE (Integrated Development Environment)` to help you with code suggestions.

In these tutorials, we are not going to use any complex form of type hints. We're basically going to attain ourselves to the simplest two forms, the (attribute, argument, etc) type, and the return types.

For attributes we use :code:`<attribute>: type`, as shown below

.. literalinclude:: ../../../../preamble/python/minimalist_package/minimalist_package/_minimalist_class.py
   :language: python
   :lines: 45

For method arguments we use :code:`<argument>: <type>` and for return types we use :code:`def <method>(<params>) -> <type>`, as shown below in our example

.. literalinclude:: ../../../../preamble/python/minimalist_package/minimalist_package/_minimalist_class.py
   :language: python
   :lines: 57-59
   :emphasize-lines: 1

Document your code with Docstrings
----------------------------------

You do not need to document every single line you code, that would in fact be quite obnoxious

.. code-block:: python

   # c stores the sum of a and b
   c = a + b

   # d stores the square of c
   d = c**2

   # check if d is zero
   if d == 0:
      # Print warning
      print("Warning")

But, on the other side of the coin, it doesn't take too long for us to forget what the parameters of a function mean. :ref:`Type hints` help a lot,
but additional information is always welcome. If you get used to using docstrings for every new method, your programming will be better in general
because documenting your code makes you think about it.

The example below shows a quick explanation of what the class does using a docstring

.. literalinclude:: ../../../../preamble/python/minimalist_package/minimalist_package/_minimalist_class.py
   :language: python
   :lines: 26-30

The `PEP 257 <https://peps.python.org/pep-0257/>`_ talks about docstrings but does not define too much beyond saying that we should use it.
My recommendation as of now would be the `Sphinx markup <https://www.sphinx-doc.org/en/master/usage/restructuredtext/domains.html#python-signatures>`_, because
of the many Python libraries using it for Sphinx documentation/tutorials like this one.

The sample code shown in this section has docstrings everywhere, but they are being used to explain the general usage of some Python syntax.
When documenting your code, obviously, the documentation should be about what the method/class/attribute does.

.. hint::

   Ideally, all documentation is perfect from the start. In reality, however, that rarely ever happens so some documentation is always better than none. 
   My advice would be to write something as it goes and possibly adjust it to more stable or cleaner documentation when the need arises.

Unit tests: always test your code
---------------------------------

.. note::

   For a comprehensive tutorial on unit testing go through the `unittest docs <https://docs.python.org/3.10/library/unittest.html>`_.

.. admonition:: In this step, we'll work on these.

   .. code-block:: console
      :emphasize-lines: 6,7
      
      python/minimalist_package/
        └── minimalist_package/
              └── __init__.py
              └── minimalist_script.py
              └── _minimalist_class.py
        └── test/
              └── test_minimalist_class.py

`Unit testing <https://en.wikipedia.org/wiki/Unit_testing>`_ is a flag that has been waved by programming enthusiasts 
and is often a good measurement of code maturity.

The elephant in the room is that writing unit tests is **boring**. Yes, we know, *very* boring.

Unit tests are boring because they are an *investment*. Unit testing won't necessarily make your
code `[...] better, faster, [...] <https://www.youtube.com/watch?v=gAjR4_CbPpQ>`_ *right now*. However, without tests, don't 
be surprised after some point if your implementations make you drown in `tech debt <https://en.wikipedia.org/wiki/Technical_debt>`_.
Dedicating a couple of minutes now to make a couple of tests when your codebase is still in its infancy makes it more manageable 
and less boresome.

Back to the example, a good practice is to create a folder name :file:`test` at the same level as the packages to be tested, like so

.. code-block::

   cd ~/ros2_tutorials_preamble/python/minimalist_package
   mkdir test

Then, we create a file named :file:`test_minimalist_class.py` with the contents below in the :file:`test` folder.

.. note::
 
   The prefix :file:`test_` is important as it is used by some frameworks to automatically discover tests. So it is better not to use
   that prefix if that file does not contain a unit test.

:download:`test_minimalist_class.py <../../../../preamble/python/minimalist_package/test/test_minimalist_class.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_package/test/test_minimalist_class.py
   :language: python
   :linenos:
   :lines: 1-

Running the tests
^^^^^^^^^^^^^^^^^

For a quick jolt of instant gratification, let's run the tests before we proceed with the explanation.

There are many ways to run tests written with :code:`unittest`. The following will run all tests found in the folder :file:`test`

.. code-block:: commandLine

   cd ~/ros2_tutorials_preamble/python/minimalist_package
   python -m unittest discover -v test

which will output

.. code-block:: commandLine
   
   test_attribute (test_minimalist_class.TestMinimalistClass) ... ok
   test_get_set_private_attribute (test_minimalist_class.TestMinimalistClass) ... ok
   test_method (test_minimalist_class.TestMinimalistClass) ... ok
   test_private_attribute (test_minimalist_class.TestMinimalistClass) ... ok
   test_static_method (test_minimalist_class.TestMinimalistClass) ... ok
   
   ----------------------------------------------------------------------
   Ran 5 tests in 0.000s
   
   OK

Yay! We've done it!

Always use :code:`unittest`
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

   ROS2 uses :code:`pytest` as default, but that doesn't mean you also have to use it in every Python code you ever write.

There are many test frameworks for Python. Nonetheless, the `unittest module <https://docs.python.org/3.10/library/unittest.html>`_
is built into Python so, unless you have a very good reason not to use it, `just [use] it <https://www.youtube.com/watch?v=ZXsQAXx_ao0>`_.

We import the :code:`unittest` module along with the class that we want to test, namely :code:`MinimalistClass`.

.. literalinclude:: ../../../../preamble/python/minimalist_package/test/test_minimalist_class.py
   :language: python
   :lines: 1-2

Test them all
^^^^^^^^^^^^^

.. note::

   Good unit tests will not only let you know when something broke but also *where* it broke. A failed test of a high-level function might not give
   you too much information, whereas a failed test of a lower-level (more fundamental) function will allow you to pinpoint the issue.

Unit tests are somewhat like insurance. The more coverage you have, the better. In this example, we test all the 
elements in the class. Each test will be based on one or more asserts. For more info check the `unittest docs <https://docs.python.org/3.10/library/unittest.html>`_.

In a few words, we make a subclass of :code:`unittest.TestCase` and create methods within it that test one part
of the code, hence the name unit tests. 

.. literalinclude:: ../../../../preamble/python/minimalist_package/test/test_minimalist_class.py
   :language: python
   :lines: 12-26

If one of the :code:`asserts` fails, then the related test will fail, and the test framework will let us know which one.

The test's main function
^^^^^^^^^^^^^^^^^^^^^^^^

Generally, a test script based on `unittest` will have the following main function. It will run all available tests in our test class.
For more info and alternatives check the `unittest docs <https://docs.python.org/3.10/library/unittest.html>`_.

.. literalinclude:: ../../../../preamble/python/minimalist_package/test/test_minimalist_class.py
   :language: python
   :lines: 29,30
   :emphasize-lines: 2
