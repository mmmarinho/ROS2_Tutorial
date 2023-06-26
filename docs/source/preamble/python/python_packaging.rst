Making your Python package installable
======================================

.. warning::

   There is some movement towards having Python deployable packages be configurable with :file:`pyproject.toml` as a default.
   However, in ROS2 and many other frameworks, the :file:`setup.py` approach using setuptools is ingrained.
   So, we'll do that for these tutorials but it doesn't necessary mean it's the best approach.

.. admonition:: In this step, we'll work on this.

   .. code-block:: console
      :emphasize-lines: 2
      
      python/minimalist_package/
        setup.py

.. literalinclude:: ../../../../preamble/python/minimalist_package/setup.py
   :language: python
   :lines: 1-
