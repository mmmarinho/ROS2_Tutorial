Frequently asked questions (FAQ)
================================

.. note::
  Also known as, frequently made comments, things I'd like to mention etc.
  

You got the name wrong, it's **ROS 2** not **ROS2**
---------------------------------------------------

Besides the humorous nature of the `meme <https://knowyourmeme.com/memes/see-nobody-cares>`_ below and my love for 1993's blockbuster, this is an inconspicuous way of showing, in every single section, that these tutorials are not official. 

.. image:: ../images/ros2_or_ros_2.jpg
   :align: center

It's not Linux, it's GNU/Linux: Keep all grievances in :code:`#vent`
--------------------------------------------------------------------

The wording on these tutorials is precise as possible. Note that some terms are commonly used with loose meanings, but I hope that the message is still conveyed. This applies to the whole tutorial, given that even official sources are not uniform in their terminology.

So, to end any deep discussions that might distract you from the point of these tutorials before they even start, I'll let you with the world-renowned Linux copypasta edited with `what was actually said <https://www.gnu.org/gnu/incorrect-quotation.html>`_ 

  *I'd just like to interject for a moment. What you're referring to as Linux, is in fact, GNU/Linux, or as I've recently taken to calling it, GNU plus Linux. Linux is not an operating system [...]. Many computer users run a modified version of the GNU system every day, without realizing it. Through a peculiar turn of events, the version of GNU which is widely used today is often called “Linux,” and many of its users are not aware that it is basically the GNU system, developed by the GNU Project. There really is a Linux, and these people are using it, but it is just a part of the system they use.*
  
  *Linux is the kernel: the program in the system that allocates the machine's resources to the other programs that you run. The kernel is an essential part of an operating system, but useless by itself; it can only function in the context of a complete operating system. Linux is normally used in combination with the GNU operating system: the whole system is basically GNU with Linux added, or GNU/Linux. All the so-called “Linux” distributions are really distributions of GNU/Linux.*

The difference between Python *scripts* and *modules*
-----------------------------------------------------

According to `The Python Tutorial on Modules <https://docs.python.org/3.10/tutorial/modules.html>`_, the definition of
*script* and *module* is not disjoint, in fact, it is said that

  *[...] you can make the file usable as a script as well as an importable module [...]*

In the official documentation, a Python script is defined as

  *[...] a [script is a] somewhat longer program, [for when] you are better off using a text editor to prepare the input for the
  interpreter and running it with [a script] as input instead [of using an interactive instance of the interpreter].*

and a module is defined as

  *[A module is a file] to put definitions [...] and use them in a script or in an interactive instance of the interpreter.*

There are more profound differences in how the Python interpreter handles *scripts* and *modules*, but in the wild the
the difference is usually as I described in :ref:`Python Terminology`.

The difference between Python *modules* and *packages*
------------------------------------------------------

According to the `Holy Book of Modules <https://docs.python.org/3.10/tutorial/modules.html>`_, a definition of packages is
given *en passant* as follows

   *Suppose you want to design a collection of modules (a “package”) [...]*

In practice, the line between modules and packages tends to be somewhat blurred. It could be a single folder with many modules but at the same time `they <https://docs.python.org/3.10/tutorial/modules.html>`_
come up with namings such as submodule

   *Packages are a way of structuring Python’s module namespace [...]. For example, the module name A.B designates a submodule named B in a package named A.*

What most people want to say when they mention a package is, usually, either a folder with a :file:`__init__.py` or a folder with a :code:`setup.py` that can be built into a :code:`wheel` or something similar. 
