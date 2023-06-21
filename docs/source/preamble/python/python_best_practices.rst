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

======  ==========
Term    Definition
script  A file that can be executed, in contrast with a file with only definitions (i.e. a library).
module  A file with content that is meant to be imported by other modules and scripts.
package A collection of modules.
======  ==========

The difference between "scripts" and "modules"
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

According to `The Python Tutorial on Modules <https://docs.python.org/3.10/tutorial/modules.html>`_, the definition of
*script* and *module* is not disjoint, in fact it is said that

::

  [...] you can make the file usable as a script as well as an importable module [...]

In the official documentation, a Python script is defined as

::

  [...] a [script is a] somewhat longer program, you are better off using a text editor to prepare the input for the
  interpreter and running it with that file as input instead.

and a module as

::

  [A module is a file] to put definitions [...] and use them in a script or in an interactive instance of the interpreter.

There are more profound differences in how the Python interpreter handles *scripts* and *modules*, but in the wild the
difference usually as I described in :ref:`Python Terminology`.


Minimalist script
-----------------

:download:`minimalist_module.py <../../../../preamble/python/minimalist_module.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_module.py
   :language: python
   :linenos:
   :lines: 24-

When a module is run directly, it's __name__ property will be '__main__'.

It is always a good idea to wrap the main() call in a try--except block
with at least the 'KeyboardInterrupt' clause. This allows the user to shutdown
the module cleanly. This is of particular importance when hardware is used,
otherwise the connection with it might be left in an undefined state causing
difficult-to-understand problems at best and physical harm at worst.

The `Exception` clause is very broad, but a MUST in code that is still under
development. Exceptions of all sorts can be generated when there is a communication
error with the hardware, software (internet etc), or other issues.
This broad clause could be replaced for a less broad exception handling if that
makes sense in a given application, but that is not necessary usually.

Minimalist Package
------------------

Minimalist class
----------------

Unit tests
----------
