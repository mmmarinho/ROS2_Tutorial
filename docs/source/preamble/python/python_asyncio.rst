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

.. hint::
   If the function/method has uses :code:`await` anywhere, it should be :code:`async` (`More info <https://peps.python.org/pep-0492/>`_).

To illustrate this, let's make a file called :file:`async_await_example.py` in :file:`minimalist_async` with the following contents.

:download:`async_await_example.py <../../../../preamble/python/minimalist_async/async_await_example.py>`

.. literalinclude:: ../../../../preamble/python/minimalist_async/async_await_example.py
   :language: python
   :linenos:
   :lines: 24-

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
