Python's :code:`asyncio`
========================

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
   If the function/method has an :code:`await`, it should be :code:`async` (`More info <https://peps.python.org/pep-0492/>`_).

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
