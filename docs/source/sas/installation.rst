Pre-requisites
--------------

1. `ROS2 Jazzy <https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html>`_
2. `dqrobotics CPP devel <https://dqroboticsgithubio.readthedocs.io/en/latest/installation/cpp.html#development-ppa>`_

.. code-block:: console

  sudo add-apt-repository ppa:dqrobotics-dev/development
  sudo apt-get update
  sudo apt-get install libdqrobotics libdqrobotics-interface-json11 libdqrobotics-interface-coppeliasim libdqrobotics-interface-coppeliasim-zmq libdqrobotics-interface-vrep

3. `dqrobotics Python devel <https://dqroboticsgithubio.readthedocs.io/en/latest/installation/python.html#installation-development>`_

.. code-block:: console

  python3 -m pip install dqrobotics --pre --break-system-packages

Create the tutorial folder
--------------------------

.. code-block:: console

  mkdir -p ~/sas_tutorial_workspace/src

Clone the sas repository
------------------------

.. code-block:: console

  cd ~/sas_tutorial_workspace/src
  git clone --recurse-submodules -b jazzy https://github.com/SmartArmStack/smart_arm_stack_ROS2.git sas

Developer environment
---------------------

It is recommended to use the latest versions of

1. `PyCharm Community <https://www.jetbrains.com/pycharm/download/?section=linux>`_
2. `QtCreator <https://www.qt.io/download-qt-installer-oss>`_
