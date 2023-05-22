Installing Python on Ubuntu
===========================

.. warning::
   If you change or try to tinker with the default Python version of Ubuntu, your system will most likely **BREAK COMPLETELY**. 
   Do not play around with the Python installation, because Ubuntu depends on it to work properly (or work at all).
   
In Ubuntu 22.04, Python is already installed!
In fact, Ubuntu would not work without it. Let's check its version by running

.. code-block:: console

   python3 --version

which should output 

.. code-block:: console

   Python 3.10.6
   
If your version is different from this, in particular, the :code:`3.10` part, get this fixed because this tutorial will not work for you. 

.. warning::
   Note that the command is :program:`python3` and not :program:`python`. In fact, the result of
   
   .. code-block:: console
   
            python
   
   is 
   
   .. code-block:: console
   
                   Command 'python' not found, did you mean:
                  command 'python3' from deb python3
                  command 'python' from deb python-is-python3

A quick Python check
====================

Run

.. code-block:: console

   python3

which should output something similar to

.. code-block:: console

   Python 3.10.6 (main, Mar 10 2023, 10:55:28) [GCC 11.3.0] on linux
   Type "help", "copyright", "credits" or "license" for more information.
   >>> 

in particular, if the :code:`[GCC 11.3.0] on linux` is different, then get this fixed because this tutorial will not work for you.

As you already know, to exit the `interative shell <https://docs.python.org/3.10/tutorial/interpreter.html>`_ you can use :kbd:`CTLR+D` or type :code:`quit()` and press :kbd:`ENTER`.

Some Python packages must be installed through :program:`apt`
=============================================================

.. warning::
   Aside from these packages that you **MUST** install from :program:`apt`, it is best to use :program:`venv` and :program:`pip` to install packages only for your user
   without using :code:`sudo`.

For some Python packages to work well with the default Python in Ubuntu, some packages must be installed through :program:`apt`. If you deviate from this you can cause issues that might not be easy to recover from.

For the purposes of this tutorial, let us install

.. code-block:: console

   sudo apt install -y python3-pip python3-venv
   




