.. include:: ../the_section_is_optional.rst

Ubuntu Terminal Basics
======================

You already know how to turn on your computer and press some keys to make bits flip and colorful pixels shine on your monitor. Here, we'll go through a few tips on Ubuntu.

.. note::

   The world is full of smart people, and they've done some amazing stuff, like Ubuntu and Linux. There are endless tutorials for those and this
   is not a complete one. In this section, we'll go through some basic tools available in Ubuntu's terminal that help with our quest to learn/use ROS2.


Who cares about the :program:`terminal` anyways, are you like 100 years old or something?
-----------------------------------------------------------------------------------------

Besides the unintended upside that if you're typing into a terminal fast enough with a black hoodie, you're cosplaying `Mr. Robot <https://www.imdb.com/title/tt4158110/>`_ at a very low cost, there wouldn't be another way to make a tutorial like this within
the current age of the Universe without relying on Ubuntu's :program:`terminal`. 

:abbr:`GUIs (Graphical User Interfaces)` change faster than long tutorials like this one can keep up with and :program:`terminal` 
is our reliable partner in crime and unlikely to change much in the foreseeable future.

For the whole tutorial, you can copy and paste the commands in :program:`terminal`. If it doesn't work, it's either your fault or mine,
but surely not the :program:`terminal`\ 's.

The :program:`terminal`
-----------------------

.. note::

   Check out `Canonical's Tutorial <https://ubuntu.com/tutorials/command-line-for-beginners>`_ on :program:`terminal` for the complete story.

.. hint:: 

   You can open a new terminal window by pressing :kbd:`CTRL+ALT+T`.

.. warning::

   This section is about the default terminal in Ubuntu 22.04. If you prefer to use some other terminal instead (there are many), then this might not be useful to you, and you might be happier referring to its documentation instead.

The :program:`terminal` is one of those things with many names. Some call it :program:`shell`, some :program:`console`, some :program:`command line`, some :program:`terminal`. I'm sure there's someone furiously typing right now saying that I'm wrong and describing in detail what those differences might be. The truth is that, in the wild (a.k.a. the Internet), those terms are used pretty much as synonyms.

For all intents and purposes, Tom Hanks is not stuck in this terminal. Instead, we use it to send commands to Ubuntu and make stuff happen.

.. list-table:: (Murilo's) List of Useful Command Line Programs
   :header-rows: 1

   * - Program
     - Example usage
     - What it does
   * - :program:`pwd`
     - :code:`pwd`
     - Outputs the absolute path to the current directory.
   * - :program:`mkdir`
     - :code:`mkdir a_folder`
     - **M**\ a\ **k**\ es a **dir**\ ectory called :file:`a_folder` in the current directory. 
   * - :program:`cd`
     - :code:`cd a_folder`
     - **C**\ hanges **d**\ irectory to a specified target.
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
     - Outputs the lines of :file:`a_file.whatever` that contain the string :code:`robocop`.
   * - :program:`nano`
     - :code:`nano a_file.whatever`
     - Helps you edit a file using a (relatively?) user-friendly program so that you don't `get stuck into vim <https://stackoverflow.blog/2017/05/23/stack-overflow-helping-one-million-developers-exit-vim/>`_.
   * - :program:`sudo`
     - :code:`sudo touch a_sudo_made_file.whatever`
     - With the powers of a **s**\ uper **u**\ ser, **do** something. It allows a given user to modify sensitive files in Ubuntu.
   * - :program:`apt`
     - :code:`sudo apt install git`
     - Installs Ubuntu packages, in this case, :program:`git`.
   * - :program:`alias`
     - :code:`alias say_hello="echo hello"`
     - Creates an alias for a command, i.e. `another way to refer to <https://dictionary.cambridge.org/dictionary/english/alias>`_ it.


Let's use it. (!?)
------------------

The thing is, we'll be using the terminal throughout the entire tutorial, so don't worry about going too deep right now.

To warm up, let's start by creating an empty file inside a new directory, as follows

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

.. _Bash redirections:

:program:`bash` redirections
----------------------------

.. info::

   More info is available at the `Bash Reference Manual <https://www.gnu.org/software/bash/manual/html_node/Redirections.html>`_.

.. hint::

   Before defaulting to writing a 300-lines-long Python script for the simplest and most common of tasks, it is always good to check if there is something already available in :program:`bash` that can do the same thing in an easier and more stable way.

In a time long long ago, before ChatGPT became the new `Deep Magic <https://www.youtube.com/watch?v=Vd6hVYkkq88>`_, :program:`bash` was already tilting heads and leaving Ubuntu users in awe.

Among many powerful features, the *redirection operator*, ``>``, stands out. It can be used to, unsurprisingly, *redirect* the output of a command to a file.

.. warning::

   The operator ``>`` overwrites the target file with the output of the preceding command, it does not ask for permission, it just goes and does it.

   The operator ``>>`` appends to the target file with the output of the preceding command.

   Don't mix these up, there is no way to undo.

For example, if we want to store the result of the command ``ls`` to a file called ``result_of_ls.txt``, the following will do

.. code-block:: console

   cd ~
   ls > result_of_ls.txt

As a default in this version of Ubuntu, if the file does not exist it is created.

Tab completion
--------------

.. hint::

   Use :kbd:`TAB` completion extensively.

Whenever I have to look at a novice's shoulders while they interact with the terminal it gives me a certain level of anxiety. That is because they are trying to perfectly type even the longest and meanest paths for files, directories, and programs.

The terminal has :kbd:`TAB` completion, so use it extensively.
You can press :kbd:`TAB` at any time to complete the name of a program, folder, file, or pretty much anything. 

For example, we can move to a folder

.. code-block:: console

   cd ~

Then type a partial command or a part of its arguments. For example,

.. code-block:: console

   rm result_o

then, by pressing :kbd:`TAB`, it should autocomplete to

.. code-block:: console

   rm result_of_ls.txt

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

I will not get into detail here about programs to change permissions because we won't need them extensively in these tutorials. However, it is important to be aware that this exists and might cause problems.

:program:`nautilus`: browsing files with a :abbr:`GUI (Graphical User Interface)`
---------------------------------------------------------------------------------

To some extent similar to :program:`explorer` in Windows and :program:`finder` in macOS, :program:`nautilus` is `the default file manager in Ubuntu <https://manpages.ubuntu.com/manpages/jammy/en/man1/nautilus.1.html>`_.

One tip is that it can be opened from the :program:`terminal` as well, so that you don't have to find whatever folder you are again.
For example, 

.. hint::

   The path :file:`.` means the current folder.

.. code-block:: console

   cd ~
   nautilus .

will open the currently logged-in user's home folder in :program:`nautilus`.
