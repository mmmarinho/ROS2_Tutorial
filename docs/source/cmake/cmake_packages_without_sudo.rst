

Install a CMake package without sudo privileges
===============================================

To install a CMake package or library without sudo privileges, we need to define a directory to which we have access. For instance, 
in :code:`~/`.


Create a custom folder
----------------------

In this tutorial, we are going to create a custom folder :code:`~/opt` containing 
the folders :code:`lib` and :code:`include`. This will be our directory to install all our CMake packages.

Run the following commands,

.. code-block:: console

    cd ~/
    mkdir -p opt && cd opt
    mkdir -p include
    mkdir -p lib

Then, we update the LD_LIBRARY_PATH, LIBRARY_PATH, and CPATH in :code:`~/.bashrc`. 

Do the following just once, so that all terminal windows automatically source this new workspace for you.

.. code-block:: console

    echo "# Update the environment variable LD_LIBRARY_PATH to include ~/opt/lib, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/opt/lib" >> ~/.bashrc

    echo "# Update the environment variable LIBRARY_PATH to include ~/opt/lib, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
    echo "export LIBRARY_PATH=$LIBRARY_PATH:~/opt/lib" >> ~/.bashrc

    echo "# Update the environment variable CPATH to include ~/opt/include, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
    echo "export CPATH=$CPATH:~/opt/include" >> ~/.bashrc

    source ~/.bashrc
  

Install a CMake package
-----------------------

To install a CMake package, we set the :code:`CMAKE_INSTALL_PREFIX:PATH` flag with our custom folder (:code:`/home/USERNAME/opt`)


.. code-block:: console

    cmake -DCMAKE_INSTALL_PREFIX:PATH=~/opt .. 
    make 
    make install


Example: Installing `ProxSuite <https://github.com/Simple-Robotics/proxsuite>`_
-------------------------------------------------------------------------------

This example shows how to build and install the ProxSuite to be used in your CMake project.

.. note:: 
  Check the `official documentation <https://github.com/Simple-Robotics/proxsuite>`_ to customize your ProxSuite installation . 


.. warning:: 
  This example assumes you have git, CMake, Eigen, and a C++ compiler installed in your GNU/Linux distribution.



.. code-block:: console

    cd ~/Downloads
    git clone https://github.com/Simple-Robotics/proxsuite.git --recursive
    cd proxsuite
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/opt -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF 
    make
    make install


Example: include and link the DQ robotics in your project
----------------------------------------------------------

.. tab-set::

    .. tab-item:: CMakeLists.txt

        :download:`CMakeLists.txt <../../../ros2_tutorial_workspace/src/cpp_cmake_example_proxsuite/CMakeLists.txt>`
        
        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_cmake_example_proxsuite/CMakeLists.txt
           :language: cmake
           :linenos:
           :emphasize-lines: 17   

    .. tab-item:: test_proxsuite.cpp

        :download:`test_dqrobotics.cpp <../../../ros2_tutorial_workspace/src/cpp_cmake_example_proxsuite/src/test_proxsuite.cpp>`

        .. literalinclude:: ../../../ros2_tutorial_workspace/src/cpp_cmake_example_dqrobotics/src/test_dqrobotics.cpp
            :language: cpp
            :linenos:
            :emphasize-lines: 2       





