Making your Python package installable
======================================

.. warning::

   There is some movement towards having Python deployable packages be configurable with :file:`pyproject.toml` as a default.
   However, in ROS2 and many other frameworks, the :file:`setup.py` approach using setuptools is ingrained.
   So, we'll do that for these tutorials but it doesn't necessary mean it's the best approach.


Use a :code:`venv`
------------------

We already know that it is a good practice to :ref:`Isolate your environment with a venv`. So, let's turn that into a reflex
and do so for this whole section.

.. code-block:: console

   cd ~
   source ros2tutorial_venv/bin/activate

The :file:`setup.py`
--------------------

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 2
      
      python/minimalist_package/
        setup.py

Has `Python Packaging <https://packaging.python.org/en/latest/>`_ ever looked daunting to you? Of course not, but let's go through a quick overview of how we can get this done.

First, we create a :file:`setup.py` at :file:`~/ros2_tutorials_preamble/python/minimalist_package` with the following contents

:download:`setup.py <../../../../preamble/python/minimalist_package/setup.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_package/setup.py
   :language: python
   :lines: 1-
   :emphasize-lines: 15-20

.. note::

   By no coincidence, the :file:`setup.py` is a Python file. We use Python to interprete it, meaning that we can processs information using
   Python to define the arguments for the :code:`setup()` function.

All arguments defined above are quite self explanatory and are passed to the :code:`setup()` function available at the :code:`setuptools` module built into Python.

The probably most unusual part of it is the :code:`entry_points` dictionary. In the key :code:`console_scripts`, we can list up scripts in the package that can be used as console programs after the package is installed. Indeed, :code:`setuptools` is rich, has a catle, and can do magic.

Installing :file:`wheel`
------------------------

.. warning::

   In the current version of Python, if you do not install :file:`wheel` as described herein, the following warning will be output.

   .. code-block:: console

      DEPRECATION: minimalist-package is being installed using the legacy 'setup.py install' method, because it does not have a 'pyproject.toml' 
      and the 'wheel' package is not installed. pip 23.1 will enforce this behaviour change. A possible replacement is to enable the '--use-pep517'
      option. Discussion can be found at https://github.com/pypa/pip/issues/8559

To install the package in the recommended way in this tutorial, we need :file:`wheel`. While using the :code:`venv`, we install it

.. code-block:: console

   python3 -m pip install wheel

Installing the Python package
-----------------------------

We first go to the folder contaning our *project* folder and we build and install the *project* filder within it using :program:`pip` as follows

.. code-block:: console

   cd ~/ros2_tutorials_preamble/python
   python3 -m pip install ./minimalist_package

which results in

.. code-block:: console

   Processing ./minimalist_package
     Preparing metadata (setup.py) ... done
   Requirement already satisfied: setuptools in ~ros2tutorial_venv/lib/python3.10/site-packages (from minimalist-package==23.6.0) (65.6.3)
   Building wheels for collected packages: minimalist-package
     Building wheel for minimalist-package (setup.py) ... done
     Created wheel for minimalist-package: filename=minimalist_package-23.6.0-py3-none-any.whl size=8608 sha256=929446a2fa81fc99fc5dec239a9f3e4439bc8fa8fe49cc4deb987d6f31b3d8b9
     Stored in directory: /private/var/folders/4k/20khytt17blf21lptscczbl00000gn/T/pip-ephem-wheel-cache-j3a0f5xy/wheels/00/16/ef/863b898c6ea4d32d47a24fda31f80cbc9cb1063742032b7d49
   Successfully built minimalist-package
   Installing collected packages: minimalist-package
   Successfully installed minimalist-package-23.6.0

Done!

Running the newly available scripts
-----------------------------------

After installing, we have access to the scripts (and packages). For instance, we can do

.. code-block:: console

   minimalist_script

which will return the friendly

.. code-block:: console

   Howdy!
   Howdy!
   Howdy!

The other two scripts are also available, for instance we can do

.. code-block:: console

   async_await_example

which will return something similar to

.. code-block:: console

   Awaiting results...
   task1 retry needed (roll = 0.1534174185325745 > 0.1).            
   task2 retry needed (roll = 0.35338687437350913 > 0.1).            
   task1 Done.
   task2 retry needed (roll = 0.3877920607121429 > 0.1).            
   The result of task=task1 was 0.07646509818952207.
   task2 retry needed (roll = 0.7010015915930288 > 0.1).            
   task2 retry needed (roll = 0.8907576123834621 > 0.1).            
   task2 retry needed (roll = 0.4233577578392548 > 0.1).            
   task2 retry needed (roll = 0.7512028176843422 > 0.1).            
   task2 retry needed (roll = 0.33501957024540663 > 0.1).            
   task2 Done.
   The result of task=task2 was 0.09239734738421612.

Importing things from the installed package
-------------------------------------------

We first run an interactive session with

.. code-block:: console

   python3 

we can then interact with is as any other installed package

.. code-block:: python

   >>> from minimalist_package import MinimalistClass
   >>> mc = MinimalistClass()
   >>> print(mc.get_private_attribute())
   20.0

Hooray!

Uninstalling packages
---------------------

Given that we installed it using :program:`pip`, removing it is also a breeze. We do

.. code-block:: console

   python3 -m pip uninstall minimalist_package

which will return something similar to

.. code-block:: console

   Found existing installation: minimalist-package 23.6.0
   Uninstalling minimalist-package-23.6.0:
     Would remove:
       /home/murilo/ros2tutorial_venv/bin/async_await_example
       /home/murilo/ros2tutorial_venv/bin/async_callback_example
       /home/murilo/ros2tutorial_venv/bin/minimalist_script
       /home/murilo/ros2tutorial_venv/lib/python3.10/site-packages/minimalist_package-23.6.0.dist-info/*
       /home/murilo/ros2tutorial_venv/lib/python3.10/site-packages/minimalist_package/*
   Proceed (Y/n)? 

and just press :kbd:`ENTER`, resulting in the package being uninstalled

.. code-block:: console

     Successfully uninstalled minimalist-package-23.6.0
