(Murilo's) ROS2 Tutorial
========================

.. note::
   
   If you're looking for the official documentation, this is **NOT** it. For the official ROS documentation, refer to this `link <https://docs.ros.org>`_.


`ROS2 Humble`_ tutorials by `yours truly <https://murilomarinho.info/>`_, focusing on Ubuntu 22.04 x64 LTS and the programming practices of successful state-of-the-art robotics implementations such as the `SmartArmStack <https://github.com/SmartArmStack>`_ and the `AISciencePlatform <https://github.com/AISciencePlatform>`_.

.. warning::

   This project is under active development and is currently just a rough draft.

Contents
--------

.. toctree::
   :caption: Preamble

   preamble/python_basics
   preamble/python_best_practices

.. toctree::
   :caption: ROS2 Setup

   installation
   terminator
   workspace_setup

.. toctree::
   :caption: ROS2 Python Package/Build Basics

   create_packages
   create_python_package
   create_python_node_with_template
   source_after_build
   
.. toctree::
   :caption: ROS2 Python Node Basics
   
   running_node
   editing_python_source
   create_python_node_from_scratch
   python_node_explained
   
.. toctree::
   :caption: ROS2 Python Library Basics   
   
   create_python_library
   using_python_library
   
.. toctree::
   :caption: ROS2 Interface Basics
   
   messages
   create_interface_package
   publishers_and_subscribers
   inspecting_topics
   service_servers_and_clients
   
.. toctree::
   :caption: Other content
   
   advanced
   faq


.. _`ROS2 Humble`: https://docs.ros.org/en/humble/
