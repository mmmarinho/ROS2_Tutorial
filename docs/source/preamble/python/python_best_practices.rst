Python Best Practices
=====================

.. warning::
   This tutorial expects prior knowledge in Python and objected-oriented programming.
   As such, this section is not meant to be a comprehensive Python tutorial. You have better resources
   made by smarter people available online, e.g. `The Python Tutorial <https://docs.python.org/3.10/tutorial/index.html>`_.

.. _Python Terminology:

Terminology
-----------

Let's go through the terminology used in this tutorial. This terminology is not necessarily uniform with other
sources/tutorials you might find elsewhere. It it is based on my interpretation of
`The Python Tutorial on Modules <https://docs.python.org/3.10/tutorial/modules.html>`_.

=======  ====================================================================================================
Term     Definition
script   A file that can be executed, in contrast with a file with only definitions (i.e. a library).
module   A file with content that is meant to be imported by other modules and scripts.
package  A collection of modules.
=======  ====================================================================================================

The difference between *scripts* and *modules*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

According to `The Python Tutorial on Modules <https://docs.python.org/3.10/tutorial/modules.html>`_, the definition of
*script* and *module* is not disjoint, in fact it is said that

  [...] you can make the file usable as a script as well as an importable module [...]

In the official documentation, a Python script is defined as

  [...] a [script is a] somewhat longer program, you are better off using a text editor to prepare the input for the
  interpreter and running it with that file as input instead.

and a module as

  [A module is a file] to put definitions [...] and use them in a script or in an interactive instance of the interpreter.

There are more profound differences in how the Python interpreter handles *scripts* and *modules*, but in the wild the
difference is usually as I described in :ref:`Python Terminology`.


Minimalist module
-----------------

Let's start with a minimalist module (in this case also a script) that prints a string periodically,
as follows. Create a file in :file:`~/ros2_tutorials_preamble/python` called `minimalist_module.py` with the following
contents.

:download:`minimalist_module.py <../../../../preamble/python/minimalist_module.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_module.py
   :language: python
   :linenos:
   :lines: 1-

Running a Python script on the terminal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

There are a few ways to run this module in the command line. Without worrying about file permissions, specifying that
the file must be interpreted by Python (and which version of Python) is the general way to run a script

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

Another way to run a Python script is to execute it directly. This can be done with

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

Note that for this second execution strategy to work, we **MUST** have the first line, called `shebang <https://en.wikipedia.org/wiki/Shebang_(Unix)>`_
which specifies what will be used to interpret that file. In general, Ubuntu does not guess the file type by the extension
when running it. The shebang must be the first line of a file, such as in our example

.. literalinclude:: ../../../../preamble/python/minimalist_module.py
   :language: python
   :linenos:
   :lines: 1

If we remove the shebang and try to execute the script, it will return the following errors, because Ubuntu doesn't know
what to do with that file.

.. code-block::

   ./minimalist_module.py: line 2: import: command not found
   ./minimalist_module.py: line 5: syntax error near unexpected token `('
   ./minimalist_module.py: line 5: `def main() -> None:'


The :code:`main` function
^^^^^^^^^^^^^^^^^^^^^^^^^

:download:`minimalist_module.py <../../../../preamble/python/minimalist_module.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_module.py
   :language: python
   :linenos:
   :lines: 12-

When a module is run directly, its :code:`__name__` property will be :code:`'__main__'`.

It is good practice to wrap the :code:`main()` call in a :code:`try--except` block
with at least the :code:`KeyboardInterrupt` clause. This allows the user to shutdown
the module cleanly either through the terminal or through :program:`PyCharm`.

This is of particular importance when hardware is used, otherwise the connection with it might be left in an undefined
state causing difficult-to-understand problems at best and physical harm at worst.

The :code:`Exception` clause in our example is very broad, but a **MUST** in code that is still under development.
Exceptions of all sorts can be generated when there is a communication error with the hardware, software (internet etc),
or other issues.

This broad :code:`Exception` clause could be replaced for a less broad exception handling if that makes sense in a given
application, but that is usually not necessary nor safe. When handling hardware, it is, in general, **IMPOSSIBLE** to
test the code of all combinations of inputs and states. As `they say <https://darkestdungeon.fandom.com/wiki/Narrator_(Darkest_Dungeon)>`_,

   Be wary, for overconfidence is a slow and insidious [source for terrible bugs and failed demos]

.. hint::

   Catching all :code:`Exceptions` might make debugging more difficult in some cases. At your own risk, you can remove
   this clause temporarily when trying to fix a stubborn bug, at the risk of forgetting to putting it back and ruining
   your hardware.

Minimalist Package
------------------

Minimalist class
----------------

Unit tests
----------
