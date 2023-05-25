.. _Asyncio:

Python's :code:`asyncio`
========================

.. note::
   Asynchronous code is not the same as code that runs in parallel, even more so in Python because of the :abbr:`GIL (Global Interpreter Lock)` (`More info <https://wiki.python.org/moin/GlobalInterpreterLock>`_).
   Basically, the :code:`async` framework allows us to not waste time waiting for results that we don't know when will arrive.
   It either allows us to attach a :code:`callback` for when the result is ready, or to run many service calls and :code:`await`
   for them all, instead of running one at a time.

There are two main ways to interact with :code:`async` code, the first being by :code:`await` -ing the results or by handling those
results through :code:`callbacks`.

Using :code:`await`
-------------------

.. note::
   **TL;DR** Using :code:`await`
   
   #. Run multiple :code:`Task`.
   #. Use :code:`await` for them, **after they were executed**.

Differently from "regular" programming, using :code:`async` needs us to reflect on several tasks being executed at the same time.
The main use case are multiple tasks that can run concurrently and, at some point, we need the result of those tasks to either
end the program or further continue with other tasks.

This type of interaction is suitable when either we need the results from all tasks before proceeding or when the order of result matters.

.. hint::
   If the function/method has uses :code:`await` anywhere, it should be :code:`async` (`More info <https://peps.python.org/pep-0492/>`_).

:download:`async_await_example.py <../../../../preamble/python/async_await_example.py>`

.. literalinclude:: ../../../../preamble/python/async_await_example.py
   :language: python
   :linenos:
   :lines: 24-

Using :code:`callback`
----------------------

.. note::
   **TL;DR** Using :code:`callbacks`
   
   #. Run multiple :code:`Task`.
   #. Add a :code:`callback` to handle the result **as soon as it is ready**.
   #. Use :code:`await` for them just so that the main loop does not die.

Differently from :code:`await` -ing for each task and then processing their result, we can define :code:`callbacks`
in such way that each result will processed as they come. In that way, the results can be processed in an arbitrary
order.

:download:`async_callback_example.py <../../../../preamble/python/async_callback_example.py>`

.. literalinclude:: ../../../../preamble/python/async_callback_example.py
   :language: python
   :linenos:
   :lines: 24-
