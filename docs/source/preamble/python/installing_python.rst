Installing Python on Ubuntu
===========================

.. warning::
   If you change or try to tinker with the default Python version of Ubuntu, your system will most likely **BREAK COMPLETELY**. 
   Do not play around with the default Python installation, because Ubuntu depends on it to work properly (or work at all).
   
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
--------------------

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
-------------------------------------------------------------

.. warning::
   Aside from these packages that you **MUST** install from :program:`apt`, it is best to use :program:`venv` and :program:`pip` to install packages only for your user
   without using :code:`sudo`.

For some Python packages to work well with the default Python in Ubuntu, they must be installed through :program:`apt`. If you deviate from this, you can cause issues that might not be easy to recover from.

For the purposes of this tutorial, let us install

.. code-block:: console

   sudo apt install -y python3-pip python3-venv
   
.. _Isolate your environment with a venv:

When you want to isolate your environment, use :program:`venv`
--------------------------------------------------------------

.. warning::
   At the time of this writing, there was no support for :program:`venv` on ROS2 `(More info) <https://github.com/ros2/ros2/issues/1094#issuecomment-897638520>`_.
   Until that is handled, we are not going to use :program:`venv` for the ROS2 tutorials. 
   However, we will use :program:`venv` to protect our ROS2 environment from these Python preamble tutorials.
 
Using :program:`venv` (`More info <https://docs.python.org/3.10/library/venv.html>`_) is quite straightforward. 

Create a :file:`venv`
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: console

   cd ~
   python3 -m venv ros2tutorial_venv
   
where the only argument, :code:`ros2tutorial_venv`, is the name of the folder in which the :code:`venv` will be created.

Activate a :file:`venv`
^^^^^^^^^^^^^^^^^^^^^^^

Whenever we want to use a :file:`venv`, it must be explicitly activated.

.. code-block:: console

   cd ~
   source ros2tutorial_venv/bin/activate
   
The terminal will change to have the prefix :code:`(ros2tutorial_venv)` to let us know that we are using a :file:`venv`, as follows

.. code-block:: console

   (ros2tutorial_venv) murilo@murilos-toaster:~$ 
   
Deactivate a :file:`venv`
^^^^^^^^^^^^^^^^^^^^^^^^^

To deactivate, run

.. code-block:: console

   deactivate

We'll know that we're no longer using the :code:`ros2tutorial_venv` because the prefix will dissapear back to 

.. code-block:: console

   murilo@murilos-toaster:~$ 

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

First, we activate the virtual environment

.. code-block:: console

   cd ~
   source ros2tutorial_venv/bin/activate

then, we install

.. code-block:: console

   python3 -m pip install dqrobotics
   
which will result in something similar to (might change depending on future versions)

.. code-block:: console

   Collecting dqrobotics
   Downloading dqrobotics-23.4.0a15-cp310-cp310-manylinux1_x86_64.whl (551 kB)
        ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 551.4/551.4 KB 6.3 MB/s eta 0:00:00
   Collecting numpy
     Downloading numpy-1.25.0-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl (17.6 MB)
        ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 17.6/17.6 MB 7.4 MB/s eta 0:00:00
   Installing collected packages: numpy, dqrobotics
   Successfully installed dqrobotics-23.4.0a15 numpy-1.25.0

Removing libraries (installed with :program:`pip`)
--------------------------------------------------

We can remove the library we just installed with

.. code-block:: console

   python3 -m pip uninstall dqrobotics

resulting in

.. code-block:: console
   
   Found existing installation: dqrobotics 23.4.0a15
   Uninstalling dqrobotics-23.4.0a15:
     Would remove:
       /home/murilo/ros2tutorial_venv/lib/python3.10/site-packages/dqrobotics-23.4.0a15.dist-info/*
       /home/murilo/ros2tutorial_venv/lib/python3.10/site-packages/dqrobotics/*
   Proceed (Y/n)?

.. hint::

   If in the terminal a question is made, the option with an uppercase letter, in this case :kbd:`Y`, will be the default.
   If you want the default, just press :kbd:`ENTER`.

Then, press :kbd:`ENTER`, which results in

.. code-block:: console

     Successfully uninstalled dqrobotics-23.4.0a15

When using :program:`pip`, do **NOT** use :code:`sudo`
------------------------------------------------------

Using :code:`sudo` without knowing what one is doing is *the* easiest way to wreak havoc in a Ubuntu installation. Even seemingly innocuous operations such as copying files with :code:`sudo` can cause irreparable damage to your Ubuntu environment.

When installing Python packages that are not available on :program:`apt`, use :program:`pip`.
