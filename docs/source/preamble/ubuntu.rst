.. include:: ../the_topic_is_under_heavy_construction.rst

Ubuntu Basics
=============

You already know how to turn your computer and press some keys to make bits flip and colorful pixels shine in your monitor. Here, we'll go through a few tips on Ubuntu.

.. note::

   The world is full of smart people, and they've done some amazing stuff, like Ubuntu and Linux. There are endless tutorials for those and this
   one is not a complete one. We'll go through some basic tools available in Ubuntu that help with our quest to learn/use ROS2.

The :program:`terminal`
-----------------------

.. note::

   Check out `Canonical's Tutorial <https://ubuntu.com/tutorials/command-line-for-beginners>`_ for the complete story.

.. warning::

   This is about the default terminal in Ubuntu 22.04. If you prefer to use the :program:`TeRmInAlDeluxeUltiMateHyruleMaster` instead, then this might not be useful to you,
   and you might be happier referring to its documentation instead.

The :program:`terminal` is one of those things with many names. Some call it :program:`shell`, some :program:`console`, some :program:`command line`, some :program:`terminal`. I'm sure there's a person way smarter than me capable of making a copypasta describing in detail what the differences might be. The truth is that, in the wild, those terms are used pretty much as synonyms.

For all intents and purposes, Tom Hanks is not stuck in this terminal. Instead, we use it to send commands to Ubuntu that make stuff happen.

.. list-table:: (Murilo's) List of Useful Command Line Programs
   :header-rows: 1

   * - Program
     - Example usage
     - What it does
   * - :program:`pwd`
     - :code:`pwd`
     - Output in the absolute path to the current directory.
   * - :program:`mkdir`
     - :code:`mkdir a_folder`
     - **M**\ a\ **k**\ es a **dir**\ ectory called :file:`this_is_a_folder` in the current directory. 
   * - :program:`cd`
     - :code:`cd a_folder`
     - **c**\ hanges **d**\ irectory to a specified target.
   * - :program:`touch`
     - :code:`touch a_file.whatever`
     - Creates an empty file called :file:`a_file.whatever`.
   * - :program:`cat`
     - :code:`cat a_file.whatever`
     - Outputs into the console the contents of :file:`a_file.whatever`.
   * - :program:`rm`
     - :code:`rm a_file.whatever`
     - **R**\ e\ **m**\ oves a file or directory (with the :code:`-r` option).
   * - :program:`ls`
     - :code:`ls`
     - **L**\ i\ **s**\ ts the contents of the current directory.
   * - :program:`grep`
     - :code:`cat a_file.whatever | grep robocop`
     - Output the lines of :file:`a_file.whatever` that contain the string :code:`robocop`.
   * - :program:`nano`
     - :code:`nano a_file.whatever`
     - Helps you do edits to a file using a (relatively?) user-friendly program, so that you don't `get stuck into vim <https://stackoverflow.blog/2017/05/23/stack-overflow-helping-one-million-developers-exit-vim/>`_.
   * - :program:`apt`
     - :code:`apt install git`
     - Installs Ubuntu packages.

Let's use it. (!?)
------------------

The thing is, we'll be using the terminal throughout the entire tutorial, so don't worry about going too deep right now.

To warm up, let's start with creating an empty file inside a new directory, as follows

.. hint::

   The path :file:`~` stands for the currently logged-in user's home folder.

.. hint:: 

   You can open a new terminal window by pressing :kbd:`CTRL+ALT+T`.

.. code-block:: console

   cd ~
   mkdir a_folder
   cd a_folder
   touch an_empty_file.txt

Then, we can use :program:`nano` to create another file with some contents

.. code-block:: console

   nano file_with_stuff.txt

Then, :program:`nano` will run. At this point we can start typing, so let's just type 

.. code-block:: console

   stuff

then you can exit with the following keys

#. :kbd:`CTRL+X`
#. :kbd:`Y`
#. :kbd:`ENTER`

you can also look at the bottom side of the window to know what keys to press. As an example, in :program:`nano`, :code:`^X` stands for :kbd:`CTRL+X`.

Then, if you run

.. code-block:: console

   ls

the output will be

.. code-block:: console

   an_empty_file.txt  file_with_stuff.txt

we can, for example, get the contents of :file:`file_with_stuff.txt` with

.. code-block:: console

   cat file_with_stuff.txt

whose output will be

.. code-block:: console

   stuff

So, enough of this example, let's get rid of everything with

.. warning::

   **ALWAYS** be careful when using :program:`rm`. `The files removed this way do NOT go to the trash can <https://unix.stackexchange.com/questions/10883/where-do-files-go-when-the-rm-command-is-issued>`_, if you use it you pretty much said `bye bye bye <https://www.youtube.com/watch?v=Eo-KmOd3i7s>`_ to those files/directories.

.. code-block:: console

   cd ~
   rm -r a_folder

Tab completion
--------------

.. hint::

   Use :kbd:`TAB` completion extensively.

Whenever I have to look at a novice's shoulders while they interact with the terminal it gives me a certain level of anxiety. The terminal has :kbd:`TAB` completion, so use it extensively.
You can press :kbd:`TAB` at any time to complete the name of a program, folder, file, or pretty much anything. 

Be careful with :program:`sudo`
-------------------------------

.. warning::

   **DO NOT**, I repeat, **DO NOT** play around with :program:`sudo`.

With great power, comes great opportunity to destroy your Ubuntu. It turns out that :program:`sudo` is the master key of destruction, it will allow you to do basically anything in the system as far as the software is concerned.

So, don't.

For these tutorials, only use :program:`sudo` when installing system-wide packages. Otherwise, do not use it.

Be careful even when not using :program:`sudo`
----------------------------------------------

With regular user privileges, the `major <https://www.youtube.com/watch?v=DDfPwaWwrII>`_ system folders will be protected from tampering. However, our home folder, e.g. :file:`/home/<YOU>` will not.
In our home folder, we are the lords, so a mistake can be fatal for your files/directories. 

File permissions
----------------

.. warning::

   **DO NOT**, I repeat, **DO NOT** play around with :program:`sudo`, :program:`chmod`, or :program:`chown`.

One of the reasons that using :program:`sudo` indiscriminately will destroy your Ubuntu is `file permissions <https://help.ubuntu.com/community/FilePermissions>`_. For example, if you *simply* open a file and save it as :program:`sudo`, you'll change its permissions, and that might be enough to even block you from logging into Ubuntu via the :abbr:`GUI (Graphics User Interface)`.

I will not get into detail here about programs to change permissions because we won't need them extensively in these tutorials. However, this is important to be aware that this exists and might cause problems.
