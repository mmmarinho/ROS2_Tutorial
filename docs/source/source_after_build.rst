.. _Always source after you build:

Always source after you build
=============================

When creating new packages or modifying existing ones, many changes will not be visible by the system unless our workspace is re-sourced.

For example, if we try the following in the terminal window we used to first build this example package

.. code :: console

   ros2 run python_package_with_a_node sample_python_node

it will not work and will output

.. code :: console

   Package 'python_package_with_a_node' not found
   
As the workspace grows bigger and the packages more complex, figuring out such errors becomes a considerable hassle. One suggestion is to always source after a build, so that sourcing errors can always be ruled out.

.. include:: the_canonical_build_command.rst
