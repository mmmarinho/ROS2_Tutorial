.. _Asyncio:

Python's :code:`asyncio`
========================

.. include:: ../../the_topic_is_under_heavy_construction.rst

.. note::
   Asynchronous code is not the same as code that runs in parallel, even more so in Python because of the :abbr:`GIL (Global Interpreter Lock)` (`More info <https://wiki.python.org/moin/GlobalInterpreterLock>`_).
   Basically, the :code:`async` framework allows us to not waste time waiting for results that we don't know when will arrive.
   It either allows us to attach a :code:`callback` for when the result is ready, or to run many service calls and :code:`await`
   for them all, instead of running one at a time.

There are two main ways to interact with :code:`async` code, the first being by :code:`await` -ing the results or by handling those
results through :code:`callbacks`. Let's go through both of them with examples.

Use a :code:`venv`
------------------

We already know that it is a good practice to :ref:`Isolate your environment with a venv`. So, let's turn that into a reflex
and do so for this whole section.

.. code-block:: console

   cd ~
   source ros2tutorial_venv/bin/activate

Create the :file:`minimalist_async` package
-------------------------------------------

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 2,3
      
      python/
        └── minimalist_async/
              └── __init__.py

As we learned in :ref:`Python package`, let's make a package called :file:`minimalist_async`.

.. code-block::

   cd ~/ros2_tutorials_preamble/python
   mkdir minimalist_async
   cd minimalist_async
   touch __init__.py

Using :code:`await`
-------------------

.. admonition:: **TL;DR** Using :code:`await`
      
   #. Run multiple :code:`Task`s.
   #. Use :code:`await` for them, **after they were executed**.

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 4
      
      python/
        └── minimalist_async/
              └── __init__.py
              └── async_await_example.py

Differently from synchronous programming, using :code:`async` needs us to reflect on several tasks being executed at the same time.
The main use case is for programs with multiple tasks that can run concurrently and, at some point, we need the result of those tasks to either
end the program or further continue with other tasks.

This type of interaction is suitable when either we need the results from all tasks before proceeding or when the order of results matters.

To illustrate this, let's make a file called :file:`async_await_example.py` in :file:`minimalist_async` with the following contents.

:download:`async_await_example.py <../../../../preamble/python/minimalist_async/async_await_example.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_async/async_await_example.py
   :language: python
   :linenos:
   :lines: 24-

Because we're going with the :code:`await` strategy, we start by defining an :code:`async` function.

.. hint::
   If the function/method has uses :code:`await` anywhere, it should be :code:`async` (`More info <https://peps.python.org/pep-0492/>`_).

.. literalinclude:: ../../../../preamble/python/minimalist_async/async_await_example.py
   :language: python
   :lines: 29-33
   :emphasize-lines: 1

The function will be run by an instance of :code:`asyncio.Task`, which basically runs the function once. To emulate us waiting for something
external without actually having to, we add a :code:`while True:` and return only with 10% chance. Instead of using a :code:`time.sleep()` we
use :code:`await asyncio.sleep(0.1)`. The main difference is that :code:`time.sleep()` is synchronous (blocking), meaning that the interpreter
will be locked here until it finishes. With :code:`await`, the interpreter is free to do other things and come back to this one later.

.. literalinclude:: ../../../../preamble/python/minimalist_async/async_await_example.py
   :language: python
   :lines: 34-47
   :emphasize-lines: 34,38,47

The example is a bit on the fancy side to make it easier to read and mantain, but it could be even simpler. When using the :code:`await` paradigm, focus on the following

#. Make the function it should run, like our :code:`unlikely_to_return()` above.
#. Run all concurrent tasks and keep a reference to them as :code:`asyncio.Task`.
#. :code:`await` on each :code:`asyncio.Task`, in the order in which you want those results.

.. literalinclude:: ../../../../preamble/python/minimalist_async/async_await_example.py
   :language: python
   :lines: 50-68
   :emphasize-lines: 3,8,46

Ok, enough with the explanation, let's go to the endorphin rush of actually running the program with

.. code-block:: commandLine

   cd ~/ros2_tutorials_preamble/python
   python -m minimalist_async.async_await_example

Which will result in something like shown below. The function is stochastic so it might take more or less to 
and the order of the tasks ending might also be different.

However, in the :code:`await` framework, the results will **ALWAYS** be processed in the order that was specified
by the :code:`await`, **EVEN WHEN THE OTHER TASK ENDS FIRST**, as in the example below. This is neither good nor bad,
it will be proper for some cases and not proper for others.

.. code-block:: commandLine
   :emphasize-lines: 21,22

   Awaiting results...
   task1 retry needed (roll = 0.36896762068176037 > 0.1).            
   task2 retry needed (roll = 0.8429002838770375 > 0.1).            
   task1 retry needed (roll = 0.841018521652675 > 0.1).            
   task2 retry needed (roll = 0.1351152094825686 > 0.1).            
   task1 retry needed (roll = 0.9484654265361889 > 0.1).            
   task2 retry needed (roll = 0.3167046796566366 > 0.1).            
   task1 retry needed (roll = 0.7519672365071198 > 0.1).            
   task2 retry needed (roll = 0.38440407016827005 > 0.1).            
   task1 retry needed (roll = 0.23155484384953284 > 0.1).            
   task2 retry needed (roll = 0.6418306170261009 > 0.1).            
   task1 retry needed (roll = 0.532161975008607 > 0.1).            
   task2 Done.
   task1 retry needed (roll = 0.448132225703992 > 0.1).            
   task1 retry needed (roll = 0.13504700640433664 > 0.1).            
   task1 retry needed (roll = 0.7404815278498079 > 0.1).            
   task1 retry needed (roll = 0.9830081693068259 > 0.1).            
   task1 retry needed (roll = 0.4070546146764875 > 0.1).            
   task1 retry needed (roll = 0.7474267487174882 > 0.1).            
   task1 Done.
   The result of task=task1 was 0.038934769861482144.
   The result of task=task2 was 0.06380247590535493.
   
   Process finished with exit code 0


Using :code:`callback`
----------------------

.. admonition:: **TL;DR** Using :code:`callbacks`
      
   #. Run multiple :code:`Task`.
   #. Add a :code:`callback` to handle the result **as soon as it is ready**.
   #. Use :code:`await` for them just so that the main loop does not die.

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 5
      
      python/
        └── minimalist_async/
              └── __init__.py
              └── async_await_example.py
              └── async_callback_example.py

Differently from :code:`await` -ing for each task and then processing their result, we can define :code:`callbacks`
in such a way that each result will be processed as they come. In that way, the results can be processed in an arbitrary
order.

:download:`async_callback_example.py <../../../../preamble/python/minimalist_async/async_callback_example.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_async/async_callback_example.py
   :language: python
   :linenos:
   :lines: 24-
