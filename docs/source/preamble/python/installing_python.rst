Installing Python on Ubuntu
===========================

.. warning::
   If you change or try to tinker with the default Python version of Ubuntu, your system will most likely **BREAK COMPLETELY**.
   Do not play around with the default Python installation, because Ubuntu depends on it to work properly (or work at all).

In Ubuntu, Python is already installed!
In fact, Ubuntu would not work without it. Let's check its version by running

.. code-block:: console

   python3 --version

which should output

.. code-block:: console

   Python 3.12.3

If the :code:`3.12` part of your version is different, this tutorial might not work for you. Please make sure to use the default Python in your Ubuntu.

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
--------------------

Run

.. code-block:: console

   python3

which should output something similar to

.. code-block:: console

   Python 3.12.3 (main, Jun 18 2025, 17:59:45) [GCC 13.3.0] on linux
   Type "help", "copyright", "credits" or "license" for more information.
   >>>

in particular, if the :code:`GCC 13` is different, then this tutorial might not work for you.

As you already know, to exit the `interactive shell <https://docs.python.org/3.12/tutorial/interpreter.html>`_ you can use :kbd:`CTRL+D` or type :code:`quit()` and press :kbd:`ENTER`.

Some Python packages must be installed through :program:`apt`
-------------------------------------------------------------

.. warning::
   Aside from these packages that you **MUST** install from :program:`apt`, it is best to use :program:`pip` to install packages only for your user
   without using :code:`sudo`.

For some Python packages to work well with the default Python in Ubuntu, they must be installed through :program:`apt`. If you deviate from this, you can cause issues that might not be easy to recover from.

For the purposes of this tutorial, let us install :code:`pip`

.. code-block:: console

   sudo apt update
   sudo apt install -y python3-pip

Installing libraries
--------------------

.. warning::
   In these tutorials, we rely either on :program:`apt` or :program:`pip` to install packages.
   There are other package managers for Python and plenty of other ways to install and manage packages.
   They are, in general, not compatible with each other so, like cleaning products, **DO NOT** mix them.

.. hint::
   Using :code:`python3 -m pip` instead of calling just :code:`pip` allows more control over which version of :program:`pip` is being called. The need for this
   becomes more evident when several Python versions have to coexist in a system.

As an example, let us install the best robot modeling and control library ever conceived, `DQ Robotics <https://github.com/dqrobotics>`_.

We install it with

.. code-block:: console

   python3 -m pip install dqrobotics --break-system-packages

which will result in something similar to (might change depending on future versions)

.. code-block:: console

    Collecting dqrobotics
      Downloading dqrobotics-25.4.0a17-cp312-cp312-manylinux2014_aarch64.whl.metadata (2.9 kB)
    Requirement already satisfied: numpy in /usr/lib/python3/dist-packages (from dqrobotics) (1.26.4)
    Downloading dqrobotics-25.4.0a17-cp312-cp312-manylinux2014_aarch64.whl (512 kB)
       ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 512.5/512.5 kB 14.0 MB/s eta 0:00:00
    [...]
    Installing collected packages: dqrobotics
    Successfully installed dqrobotics-25.4.0a17

Removing libraries (installed with :program:`pip`)
--------------------------------------------------

We can remove the library we just installed with

.. code-block:: console

   python3 -m pip uninstall dqrobotics --break-system-packages

resulting in

.. code-block:: console

    Found existing installation: dqrobotics 25.4.0a7
    Uninstalling dqrobotics-25.4.0a7:
      Would remove:
        /usr/local/lib/python3.12/dist-packages/dqrobotics-25.4.0a7.dist-info/*
        /usr/local/lib/python3.12/dist-packages/dqrobotics/*
    Proceed (Y/n)?

.. hint::

   If in the terminal a question is made, the option with an uppercase letter, in this case :kbd:`Y`, will be the default.
   If you want the default, just press :kbd:`ENTER`.

Then, press :kbd:`ENTER`, which results in

.. code-block:: console

     Successfully uninstalled dqrobotics-25.4.0a7

When using :program:`pip`, do **NOT** use :code:`sudo`
------------------------------------------------------

Using :code:`sudo` without knowing what one is doing is *the* easiest way to wreak havoc in a Ubuntu installation. Even seemingly innocuous operations such as copying files with :code:`sudo` can cause irreparable damage to your Ubuntu environment.

When installing Python packages that are not available on :program:`apt`, use :program:`pip`.
