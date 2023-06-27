Ubuntu Basics
=============

You already know how to turn your computer and press some keys to make bits flip and colorful pixels shine in your monitor. Here, we'll go through a few tips

.. note::

   The world is full of smart people, and they've done some amazing stuff, like Ubuntu and Linux. There are endless tutorials for those and this
   one is not a complete one. We'll go through some basic tools available in Ubuntu that help with our quest to learn/use ROS2.

.. include:: ../the_topic_is_under_heavy_construction.rst

Keep all grievances in :code:`#vent`
------------------------------------

The wording on these tutorials is precise as much as possible. Note that some terms are commonly used with loose meanings, but the message is still conveyed. 

So, to end any deep discussions that might distract you from the point of these tutorials before they even start, I'll let you with the world-renowned Linux copypasta edited with `what was actually said <https://www.gnu.org/gnu/incorrect-quotation.html>`_ 

  *I'd just like to interject for a moment. What you're referring to as Linux, is in fact, GNU/Linux, or as I've recently taken to calling it, GNU plus Linux. Linux is not an operating system [...]. Many computer users run a modified version of the GNU system every day, without realizing it. Through a peculiar turn of events, the version of GNU which is widely used today is often called “Linux,” and many of its users are not aware that it is basically the GNU system, developed by the GNU Project. There really is a Linux, and these people are using it, but it is just a part of the system they use.*
  
  *Linux is the kernel: the program in the system that allocates the machine's resources to the other programs that you run. The kernel is an essential part of an operating system, but useless by itself; it can only function in the context of a complete operating system. Linux is normally used in combination with the GNU operating system: the whole system is basically GNU with Linux added, or GNU/Linux. All the so-called “Linux” distributions are really distributions of GNU/Linux.*

The :program:`terminal`
-----------------------

.. note::

   Check out `Canonical's Tutorial <https://ubuntu.com/tutorials/command-line-for-beginners>`_ for the complete story.

The :program:`terminal` is one of those things with many names. Some call it :program:`shell`, some :program:`console`, some :program:`command line`, some :program:`terminal`. I'm sure there's a person way smarter than me capable of making a copypasta describing in detail what the differences might be. The truth is that, in the wild, those terms are used pretty much as synonyms.

In this terminal, Tom Hanks is not stuck in an airport. Instead, we use it to send commands to Ubuntu that make stuff happen.

.. list-table:: (Murilo's) List of Useful Command Line Programs
   :header-rows: 1

   * - Program
     - Example usage
     - What it does
   * - :program:`pwd`
     - :code:`pwd`
     - Output in the absolute path to the current directory.
   * - :program:`mkdir`
     - :code:`mkdir this_is_a_folder`
     - Makes a folder called :file:`this_is_a_folder` in the current directory. 
   * - :program:`cat`
     - :code:`cat a_file.whatever`
     - Outputs into the console the contents of :file:`a_file.whatever`.
   * - :program:`ls`
     - :code:`ls`
     - Outputs the contents of the current directory.
   * - :program:`touch`
     - :code:`touch new_empty_file.yo`
     - Creates an empty file called :file:`new_empty_file.yo`.
   * - :program:`grep`
     - :code:`cat a_file.whatever | grep robocop`
     - Output the lines of :file:`a_file.whatever` that contain the string :code:`robocop`.
