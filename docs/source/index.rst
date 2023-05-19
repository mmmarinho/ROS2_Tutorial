(Murilo's) ROS2 Tutorial
========================

.. note::
   
   If you're looking for the official documentation, this is **NOT** it. For the official ROS documentation, refer to this `link <https://docs.ros.org>`_.


`ROS2 Humble`_ tutorials by `yours truly <https://murilomarinho.info/>`_, focusing on Ubuntu 22.04 x64 LTS and the programming practices of successful state-of-the-art robotics implementations such as the `SmartArmStack <https://github.com/SmartArmStack>`_ and the `AISciencePlatform <https://github.com/AISciencePlatform>`_.

.. warning::

   This project is under active development and is currently just a rough draft.

Using this tutorial
-------------------

This is a tutorial that supposes that the user will follow it linearly. 

:doc:`Preamble <preamble/python_basics>`
    Basic content such as simple Ubuntu use, Python in Ubuntu etc.
    
:doc:`ROS2 Setup <installation>`
    Setting up a working ROS2 enviroment.

:doc:`ROS2 Tutorial`
    The first lesson of the tutorial, supposing that the ROS2 enviroment is properly set up.

.. toctree::
   :caption: Preamble
   :hidden:

   preamble/python_basics
   preamble/python_best_practices

.. toctree::
   :caption: ROS2 Setup
   :hidden:

   installation
   terminator
   workspace_setup

.. toctree::
   :caption: ROS2 Python Package/Build Basics
   :hidden:

   create_packages
   create_python_package
   create_python_node_with_template
   source_after_build
   
.. toctree::
   :caption: ROS2 Python Node Basics
   :hidden:
   
   running_node
   editing_python_source
   create_python_node_from_scratch
   python_node_explained
   
.. toctree::
   :caption: ROS2 Python Library Basics
   :hidden:
   
   create_python_library
   using_python_library
   
.. toctree::
   :caption: ROS2 Interface Basics
   :hidden:
   
   messages
   create_interface_package
   publishers_and_subscribers
   inspecting_topics
   service_servers_and_clients
   
.. toctree::
   :caption: Other content
   :hidden:
   
   advanced
   faq


.. _`ROS2 Humble`: https://docs.ros.org/en/humble/
