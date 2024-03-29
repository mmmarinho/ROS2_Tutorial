.. _ROS2 installation:

ROS2 Installation
=================

.. note:: 
  This tutorial is an abridged version of the original `ROS 2 Documentation <https://docs.ros.org/en/humble/index.html>`_. This tutorial considers a fresh Ubuntu Desktop (not Server) 22.04 LTS x64 (not arm64) installation, that you have super user access and common sense. It might work in other cases, but those have not been tested in this tutorial.

.. warning:: 
  All commands must be followed to the letter, in the precise order described herein. Any deviation from what is described might cause unspecified problems and not all of them are easily solvable.

Update :program:`apt` packages
------------------------------

.. hint:: 
  You can quickly open a new terminal window by pressing :kbd:`CTRL+ATL+T`.

After a fresh install, update and upgrade all :program:`apt` packages.

.. code-block:: console

   sudo apt update && sudo apt upgrade -y


Install a few pre-requisites
----------------------------

.. code-block:: console

   sudo apt install -y software-properties-common curl terminator git
   
Namely:

===========================   ================================================================================================================================================
software-properties-common_   Allows us to access the ROS2 packages using :program:`apt`.
curl_                         Helps download installation/configuration files from the terminal.
terminator_                   ROS uses plenty of terminals, so this helps keep one's sanity intact by enabling the management of several terminals in a single window. Despite what some might say, this particular terminator has no interest whatsoever in Sarah Connor.
git_                          The trendy source control program everyone mentions in their CV. You might be interested in knowing why it's called :code:`git`.
===========================   ================================================================================================================================================

Add ROS2 sources
----------------

Your :program:`apt` needs to know where the ROS2 packages can be found and to be able to verify their authenticity. After setting up the :program:`apt` sources, the local package list must be updated.
The following commands will do all that magic.

.. code-block:: console

   sudo add-apt-repository universe
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update && sudo apt upgrade -y

Install ROS2 packages
---------------------

There are plenty of ways to install ROS2, the following will suffice for now. 

.. code-block:: console

   sudo apt install -y ros-humble-desktop ros-dev-tools 

Set up system environment to find ROS2
-------------------------------------

ROS2 packages are implemented in such a way that they live peacefully in the :code:`/opt/ros/{ROS_DISTRO}` folder in your Ubuntu. A given terminal window or program will only know that ROS2 exists, and which version you want to use, if you run a setup file *for each terminal, every time you open a new one*.

The :code:`~/.bashrc` file can be used for that exact purpose as, in Ubuntu, that is the file that configures each terminal window for a given user.

**TL;DR** just run this **ONCE AND ONLY ONCE**

.. code-block:: console

   echo "# Source ROS2 Humble, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   
Check if it works
-----------------

If the following command

.. code-block:: console
    
   ros2

outputs something similar to what is shown below, then it worked! Otherwise, it didn't!

.. code-block:: console

       usage: ros2 [-h] [--use-python-default-buffering]
                Call `ros2 <command> -h` for more detailed usage. ...

    ros2 is an extensible command-line tool for ROS 2.

    options:
      -h, --help            show this help message and exit
      --use-python-default-buffering
                            Do not force line buffering in stdout and instead use
                            the python default buffering, which might be affected
                            by PYTHONUNBUFFERED/-u and depends on whatever stdout
                            is interactive or not

    Commands:
      action     Various action related sub-commands
      bag        Various rosbag related sub-commands
      component  Various component related sub-commands
      daemon     Various daemon related sub-commands
      doctor     Check ROS setup and other potential issues
      interface  Show information about ROS interfaces
      launch     Run a launch file
      lifecycle  Various lifecycle related sub-commands
      multicast  Various multicast related sub-commands
      node       Various node related sub-commands
      param      Various param related sub-commands
      pkg        Various package related sub-commands
      run        Run a package specific executable
      security   Various security related sub-commands
      service    Various service related sub-commands
      topic      Various topic related sub-commands
      wtf        Use `wtf` as alias to `doctor`

      Call `ros2 <command> -h` for more detailed usage.



.. _software-properties-common: https://askubuntu.com/questions/1000118/what-is-software-properties-common
.. _curl: https://curl.se/
.. _terminator: https://manpages.ubuntu.com/manpages/bionic/man1/terminator.1.html
.. _git: https://en.wikipedia.org/wiki/Git
