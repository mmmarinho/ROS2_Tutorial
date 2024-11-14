

Install a CMake package without sudo privileges
===============================================

To install a CMake package or library without sudo privileges, we need to define a directory to which we have access. For instance, 
in :code:`~/`.


Create a custom folder
----------------------

In this tutorial, we are going to create a custom folder :code:`~/opt` containing 
the folders :code:`lib` and :code:`include`. This will be our directory to install all our CMake packages.


.. code-block:: console

    cd ~/
    mkdir -p opt && cd opt
    mkdir -p include
    mkdir -p lib

Then, we update the LD_LIBRARY_PATH in :code:`~/.bashrc`.    

.. code-block:: console

    echo "# Update the environment variable LD_LIBRARY_PATH to include ~/opt/lib, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/opt/lib" >> ~/.bashrc
    source ~/.bashrc


.. image:: ../../images/cmake_without_sudo_steps.gif
   :align: center    


Install a CMake package
-----------------------

To install a CMake package, we set the :code:`CMAKE_INSTALL_PREFIX:PATH` flag with our custom folder (:code:`/home/USERNAME/opt`)


.. code-block:: console

    cmake -DCMAKE_INSTALL_PREFIX:PATH=~/opt .. 
    make 
    make install


Example: DQ robotics library
-----------------------------

This example shows how to build and install the DQ robotics library to be used in your CMake project.

.. note:: 
  If you use an active Ubuntu LTS version and have sudo access, you can install the DQ Robotics library using a few
  commands in the terminal. Check this [link](https://dqrobotics.github.io/). 


.. warning:: 
  This example assumes you have git, CMake, Eigen, and a C++ compiler installed in your GNU/Linux distribution.



.. code-block:: console

    git clone https://github.com/dqrobotics/cpp.git
    cd cpp
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX:PATH=~/opt .. 
    make 
    make install


Example: include and link the DQ robotics in your project
----------------------------------------------------------

.. tab-set::

    .. tab-item:: CMakeLists.txt

        :download:`CMakeLists.txt <../../../ros2_tutorial_workspace/src/cpp_cmake_example_dqrobotics/CMakeLists.txt>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_cmake_example_dqrobotics/CMakeLists.txt
           :language: cmake
           :linenos:
           :emphasize-lines: 14,16,17   

    .. tab-item:: test_dqrobotics.cpp

        :download:`test_dqrobotics.cpp <../../../ros2_tutorial_workspace/src/cpp_cmake_example_dqrobotics/src/test_dqrobotics.cpp>`

        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_cmake_example_dqrobotics/src/test_dqrobotics.cpp
            :language: cpp
            :linenos:
            :emphasize-lines: 2       





