========================
(Murilo's) ROS2 Tutorial
========================

.. note::
   If you're looking for the official documentation, this is **NOT** it. For the official ROS documentation, refer to this `link <https://docs.ros.org>`_.

.. hint::

   You can download this tutorial as a `PDF <https://ros2-tutorial.readthedocs.io/_/downloads/en/latest/pdf/>`_ 📀.

📖 **About this tutorial** 

`ROS2 Humble <https://docs.ros.org/en/humble/>`_ tutorials by `Murilo M. Marinho <https://murilomarinho.info/>`_, focusing on Ubuntu 22.04 x64 LTS and the programming practices of successful state-of-the-art robotics implementations such as the `SmartArmStack <https://github.com/SmartArmStack>`_ and the `AISciencePlatform <https://github.com/AISciencePlatform>`_.

🤟 **Using this tutorial** 

This is a tutorial that supposes that the user will follow it linearly. Some readers can skip the :doc:`Preamble <preamble/ubuntu>` if they are somewhat already comfortable in Python and Ubuntu. Otherwise, all steps can be considered as dependent on the prior ones, starting from :doc:`ROS2 Setup <installation>`. 

🔍 **Quick overview**

#. :doc:`Preamble: Ubuntu Basics <preamble/ubuntu>`
    A few tips on Ubuntu/terminal usage.

#. :doc:`Preamble: Python Basics <preamble/python>` 
    A quick memory refresher for the Python stuff we'll use in ROS2.

#. :doc:`ROS2 Setup <installation>` (⭐start here⭐)
    Installing ROS2 and setting up its environment for use.

#. :doc:`ROS2 Python Package/Build Basics <create_packages>`
    Creating our first ROS2 package with :program:`ament_python` and building it with :program:`colcon`.

#. :doc:`ROS2 Python Node Basics <running_node>`
    Creating a ``rclpy`` Node and figuring out what all that means.

#. :doc:`ROS2 Python Library Basics <create_python_library>`
    Create a Python library and importing/using it in another :program:`ament_python` package.

#. :doc:`ROS2 Python Interface Basics <messages>`
    Making ROS2 messages, services, publishers, subscribers, service servers, and service clients.

#. :doc:`ROS2 Parameter/Launch Basics <parameters_and_launch>`
    Making configurable ROS2 Nodes using parameters and launch files.

.. toctree::
   :caption: Preamble
   :maxdepth: 2
   :hidden:

   preamble/ubuntu
   preamble/python
    
.. toctree::
   :caption: ROS2 Setup (⭐start here⭐)
   :maxdepth: 2
   :hidden:

   installation
   terminator
   workspace_setup

.. toctree::
   :caption: ROS2 Python Package/Build Basics
   :maxdepth: 2
   :hidden:

   create_packages
   create_python_package
   create_python_node_with_template
   source_after_build

.. toctree::
   :caption: ROS2 Python Node Basics
   :maxdepth: 2
   :hidden:
   
   running_node
   editing_python_source
   create_python_node_from_scratch
   python_node_explained

.. toctree::
   :caption: ROS2 Python Library Basics
   :maxdepth: 2
   :hidden:
   
   create_python_library
   using_python_library

.. toctree::
   :caption: ROS2 Python Interface Basics
   :maxdepth: 2
   :hidden:
   
   messages
   create_interface_package
   publishers_and_subscribers
   inspecting_topics
   service_servers_and_clients
   inspecting_services

.. toctree::
   :caption: ROS2 Parameter/Launch Basics
   :maxdepth: 2
   :hidden:

   parameters_and_launch
   launch_configurable_nodes
   inspecting_parameters

.. toctree::
   :caption: Other content
   :maxdepth: 2
   :hidden:
   
   advanced
   faq

Warnings
--------

.. warning::
   If you're using macOS or Windows, this is **NOT** the guide for you. There might be a lot of overlap, but none of the code shown here has been tested on those operating systems.

.. warning::
   This project is under active development and is currently a draft.

Disclaimers
-----------

.. admonition:: `Disclaimer <https://www.southparkstudios.com/legal/thmcbs/show-disclaimer>`_

   ANYTHING ON THIS TUTORIAL--EVEN THINGS THAT ACTUALLY WORK--IS ENTIRELY FICTIONAL. SOME MEMES ARE ATTEMPTED....POORLY. THE TUTORIAL CONTAINS MISPLACED MOVIE REFERENCES AND DUE TO ITS LOW-HANGING FRUIT HUMOUR, IT SHOULD NOT BE READ BY ANYONE.

.. admonition:: Disclaimer

   All advice, comments, and terrible memes in this tutorial are my own and not endorsed by anyone or anything else mentioned herein. 
   
   THIS TUTORIAL AND RELATED SOFTWARE ARE PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE AND/OR TUTORIAL, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   
   By reading and/or using the tutorial in total or in part, you agree to these terms.
