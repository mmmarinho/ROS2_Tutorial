========================
(Murilo's) ROS2 Tutorial
========================

.. note::
   If you're looking for the official documentation, this is **NOT** it. For the official ROS documentation, refer to this `link <https://docs.ros.org>`_.

.. hint::

   You can download this tutorial as a `PDF <https://ros2-tutorial.readthedocs.io/_/downloads/en/latest/pdf/>`_ 📀.


🧑‍⚖️ **License**

.. image:: ../images/by-nc-nd.png
   :width: 150
   :alt: License shield
\

This tutorial is licensed under `Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International <https://creativecommons.org/licenses/by-nc-nd/4.0/>`_.

The example code have their own License headers, usually `MIT Licensed <https://opensource.org/license/mit>`_.

.. attention:: AI training with this content must follow the `license restrictions <https://creativecommons.org/using-cc-licensed-works-for-ai-training-2/>`_.
   In particular, please refer to the excerpt below.

   *If AI training data includes the NonCommercial restriction,
   then following the NC restriction would require that all stages,
   from copying the data during training to sharing the trained model, must not be for commercial gain.*

📖 **About this tutorial** 

`ROS2 Jazzy <https://docs.ros.org/en/jazzy/>`_ tutorials by `Murilo M. Marinho <https://murilomarinho.info/>`_, focusing on Ubuntu 24.04 x64 LTS and the programming practices of successful state-of-the-art robotics implementations such as the `SmartArmStack <https://smartarmstack.github.io>`_ also used in the `AISciencePlatform <https://github.com/AISciencePlatform>`_.

These tutorials have been the backbone of `EEEN62021 Software for Robotics <https://www.manchester.ac.uk/study/masters/courses/list/20967/msc-robotics/course-details/EEEN62021>`_, one of the units of the `MSc Robotics <https://www.manchester.ac.uk/study/masters/courses/list/20967/msc-robotics/>`_ at the `University of Manchester <https://www.manchester.ac.uk>`_.

🤟 **Using this tutorial** 

This is a tutorial that supposes that the user will follow it linearly. Some readers can skip the :doc:`Preamble <preamble/ubuntu>` if they are somewhat already comfortable in Python and Ubuntu. Otherwise, all steps can be considered as dependent on the prior ones, starting from :doc:`ROS2 Setup <installation>`.

It is expected that the user will be working directly on an Ubuntu machine. :program:`Docker` images are available but no compatibility with host systems other than Ubuntu have been attempted.

❤️ **Ways to show love**

If you enjoyed this tutorial, please

- ⭐ star `the repository <https://github.com/mmmarinho/ROS2_Tutorial>`_ or,
- contribute with feedback to the `issue tracker <https://github.com/mmmarinho/ROS2_Tutorial/issues>`_.

🔍 **Quick overview**

#. :doc:`Preamble: Ubuntu and Python <preamble/ubuntu>`
    A few tips on Ubuntu terminal and checking your Python installation.

#. :doc:`Preamble: Python Best Practices <preamble/python/python_best_practices>`
    A quick memory refresher for the Python stuff useful in ROS2.

#. :doc:`ROS2 Setup <installation>` (⭐start here⭐)
    Installing ROS2 and setting up its environment for use.

#. :doc:`ROS2 Python Package/Build <create_packages>`
    Creating our first ROS2 package with :program:`ament_python` and building it with :program:`colcon`.

#. :doc:`ROS2 Python Node <running_node>`
    Creating ``rclpy`` Nodes and figuring out what all that means.

#. :doc:`ROS2 Python Library <create_python_library>`
    Create Python librarys and importing/using it in another :program:`ament_python` package.

#. :doc:`ROS2 Interfaces <interfaces>`
    ROS2 interfaces, i.e., messages ``.msg``, services ``.srv``, and actions ``.action``. Creating custom interface packages with :program:`ament_cmake`.

#. :doc:`ROS2 Messages in Python <publishers_and_subscribers>`
    Writing and using ROS2 messages, publishers, and subscribers.

#. :doc:`ROS2 Parameter/Launch <parameters_and_launch>`
    Making configurable ROS2 Nodes using parameters and launch files.

#. :doc:`ROS2 Services in Python <service_servers_and_clients>`
    Writing ROS2 services, service servers, and service clients.

#. :doc:`ROS2 Actions in Python <actions>`
    Writing ROS2 actions, action servers, and action clients.

#. :doc:`Frame Transformations <transformations/index>`
    Frame transformations, the ``TransformStamped`` message, and ``tf2``.

#. :doc:`Gazebo and ROS2 <gazebo/index>`
    :program:`Gazebo` installation, basic usage, and integration examples with ROS2.

.. toctree::
   :caption: Preamble
   :maxdepth: 2
   :hidden:

   preamble/ubuntu
   preamble/python/installing_python
   preamble/python/python_best_practices
   preamble/python/python_asyncio
   preamble/python/python_packaging
    
.. toctree::
   :caption: ROS2 Setup (⭐start here⭐)
   :maxdepth: 2
   :hidden:

   installation
   terminator
   workspace_setup

.. toctree::
   :caption: ROS2 Python Package/Build
   :maxdepth: 2
   :hidden:

   create_packages
   create_python_package
   create_python_node_with_template
   source_after_build
   running_node

.. toctree::
   :caption: ROS2 Python Node
   :maxdepth: 2
   :hidden:
   
   create_python_node_from_scratch
   python_node_explained

.. toctree::
   :caption: ROS2 Python Library
   :maxdepth: 2
   :hidden:
   
   create_python_library
   using_python_library

.. toctree::
   :caption: ROS2 Interfaces: Messages, Services, and Actions
   :maxdepth: 2
   :hidden:

   interfaces
   create_interface_package

.. toctree::
   :caption: ROS2 Messages in Python
   :maxdepth: 2
   :hidden:
   
   publishers_and_subscribers
   inspecting_topics

.. toctree::
   :caption: ROS2 Parameter/Launch
   :maxdepth: 2
   :hidden:

   parameters_and_launch
   launch_configurable_nodes
   inspecting_parameters

.. toctree::
   :caption: ROS2 Services in Python
   :maxdepth: 2
   :hidden:

   service_servers_and_clients
   inspecting_services

.. toctree::
   :caption: ROS2 Actions in Python
   :maxdepth: 2
   :hidden:

   actions
   action_servers
   action_clients

.. toctree::
   :caption: Frame transformations and tf2
   :maxdepth: 2
   :hidden:

   transformations/index
   transformations/tf2
   transformations/rviz
   transformations/nottf2

.. toctree::
   :caption: Gazebo
   :maxdepth: 2
   :hidden:

   gazebo/index
   gazebo/installation
   gazebo/usage
   gazebo/ros_gz_bridge
   gazebo/custom_nodes

.. toctree::
   :caption: ROS2 C++ Basics
   :maxdepth: 2
   :hidden:
   
   cpp/cpp_node
   cpp/cpp_library
   cpp/cpp_vent

.. toctree::
   :caption: SAS Basics
   :maxdepth: 2
   :hidden:
   
   sas/index
   sas/installation
   sas/sas_robot_driver_add_new_robot

.. toctree::
   :caption: Other content
   :maxdepth: 2
   :hidden:

   docker/index
   cmake/index
   faq

.. toctree::
   :caption: Unstable
   :maxdepth: 2
   :hidden:

   navigation/index


Disclaimers
-----------

By reading and/or using this tutorial in total or in part, you agree to these terms.

.. admonition:: `Disclaimer <https://www.southparkstudios.com/legal/thmcbs/show-disclaimer>`_

   ANYTHING ON THIS TUTORIAL--EVEN THINGS THAT ACTUALLY WORK--IS ENTIRELY FICTIONAL. SOME MEMES ARE ATTEMPTED....POORLY. THE TUTORIAL CONTAINS MISPLACED MOVIE REFERENCES AND DUE TO ITS LOW-HANGING FRUIT HUMOUR, IT SHOULD NOT BE READ BY ANYONE.

.. admonition:: Disclaimer

   All advice, comments, and terrible memes in this tutorial are my own and not endorsed by anyone or anything else mentioned herein. It's not even endorsed by me.

.. admonition:: Disclaimer

   THIS TUTORIAL AND RELATED SOFTWARE ARE PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE AND/OR TUTORIAL, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   
Changelog
---------

2025/08-2025/12
+++++++++++++++

- Updated to ROS2 Jazzy.
- Updated theme to ``sphinx_book_theme``.
- Moved ROS2 parameter and launch to before the services.
- Added a section mostly about ROS2 Actions.
- Added a section mostly about ``tf2``.
