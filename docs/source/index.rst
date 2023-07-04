(Murilo's) ROS2 Tutorial
========================

.. note::
   If you're looking for the official documentation, this is **NOT** it. For the official ROS documentation, refer to this `link <https://docs.ros.org>`_.

`ROS2 Humble`_ tutorials by `Murilo M. Marinho <https://murilomarinho.info/>`_, focusing on Ubuntu 22.04 x64 LTS and the programming practices of successful state-of-the-art robotics implementations such as the `SmartArmStack <https://github.com/SmartArmStack>`_ and the `AISciencePlatform <https://github.com/AISciencePlatform>`_.

Using this tutorial
-------------------

This is a tutorial that supposes that the user will follow it linearly. Some readers can skip the :doc:`Preamble <preamble/ubuntu>` if they are somewhat already comfortable in Python and Ubuntu. Otherwise, all steps can be considered as dependent on the prior ones, starting from :doc:`ROS2 Setup <installation>`. 

Preamble
--------

Basic content to refresh your memory, such as simple Ubuntu use, Python in Ubuntu, etc.

.. toctree::
   :caption: Preamble
   :hidden:

   preamble/ubuntu
   preamble/python
    
ROS2 Setup (⭐The ROS2 tutorial starts here⭐)
-----------------------------------------------

Setting up a working ROS2 environment properly to follow this tutorial. 

.. toctree::
   :caption: ROS2 Setup

   installation
   terminator
   workspace_setup

ROS2 Python Package/Build Basics
--------------------------------

After setting up, instructions on creating and building your first package and how to build it.

.. toctree::
   :caption: ROS2 Python Package/Build Basics

   create_packages
   create_python_package
   create_python_node_with_template
   source_after_build

ROS2 Python Node Basics
-----------------------

Instructions on how to create a ROS2 Python node from scratch and how it works.

.. toctree::
   :caption: ROS2 Python Node Basics
   
   running_node
   editing_python_source
   create_python_node_from_scratch
   python_node_explained

ROS2 Python Library Basics
--------------------------

Instructions on how to make a ROS2 Python library to use in another package.

.. toctree::
   :caption: ROS2 Python Library Basics
   
   create_python_library
   using_python_library

ROS2 Python Interface Basics
----------------------------

ROS2 interfaces, such as messages and services, how to define them and use them in publishers, subscribers, service servers, and service clients.

.. toctree::
   :caption: ROS2 Python Interface Basics
   
   messages
   create_interface_package
   publishers_and_subscribers
   inspecting_topics
   service_servers_and_clients
   inspecting_services

ROS2 Parameter/Launch Basics
----------------------------

Working with ROS2 parameters and launch files for configurable ROS2 Nodes.

.. toctree::
   :caption: ROS2 Parameter/Launch Basics

   parameters_and_launch
   launch_configurable_nodes
   inspecting_parameters

Other content
-------------

.. toctree::
   :caption: Other content
   
   advanced
   faq

Disclaimers etc
---------------

.. warning::
   This project is under active development and is currently a draft.

.. warning::
   If you're using macOS or Windows, this is **NOT** the guide for you. There might be a lot of overlap, but none of the code shown here has been tested on those operating systems.

.. admonition:: Disclaimer

   All advice, comments, and terrible memes in this tutorial are my own and not endorsed by anyone or anything else mentioned herein. 

   THIS TUTORIAL AND RELATED SOFTWARE ARE PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE AND/OR TUTORIAL, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   By reading and/or using the tutorial in total or in part, you agree to these terms.

.. _`ROS2 Humble`: https://docs.ros.org/en/humble/
