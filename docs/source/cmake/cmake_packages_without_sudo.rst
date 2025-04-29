

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


Example: Installing `qpOASES <https://github.com/coin-or/qpOASES>`_
-------------------------------------------------------------------------------

This example shows how to build and install the qpOASES to be used in your CMake project.

.. note:: 
  Check the `official qpOASES documentation <https://github.com/coin-or/qpOASES>`_ for more details. 


.. warning:: 
  This example assumes you have git, CMake, Eigen, and a C++ compiler installed in your GNU/Linux distribution.


To install qpOASES as a shared library, we use the instructions provided by the DQ Robotics in \
`cpp-interface-qpoases <https://github.com/dqrobotics/cpp-interface-qpoases>`_ specifying the
installation directory. 

.. code-block:: console

    cd ~/Downloads
    git clone https://github.com/coin-or/qpOASES.git
    cd qpOASES
    sed -i -e 's/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" OFF)/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" ON)/g' CMakeLists.txt
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/opt
    make 
    make install


Example: include and link the qpOASES in your project
-------------------------------------------------------

.. tab-set::

    .. tab-item:: CMakeLists.txt

        :download:`CMakeLists.txt <../../../cmake_tutorial_workspace/src/cpp_cmake_example_qpoases_lib/CMakeLists.txt>`
        
        .. literalinclude:: ../../../cmake_tutorial_workspace/src/cpp_cmake_example_qpoases_lib/CMakeLists.txt
           :language: cmake
           :linenos:
           :emphasize-lines: 17   

    .. tab-item:: test_qpoases.cpp

        :download:`test_dqrobotics.cpp <../../../cmake_tutorial_workspace/src/cpp_cmake_example_qpoases_lib/src/test_qpoases.cpp>`

        .. literalinclude:: ../../../cmake_tutorial_workspace/src/cpp_cmake_example_qpoases_lib/src/test_qpoases.cpp
            :language: cpp
            :linenos:
            :emphasize-lines: 2,3   


.. warning:: 
  If you have the library installed in two directories, you need to ensure you are linking the library you want. 

For instance, let's say you have the DQ Robotics library installed globally (i.e., :code:`/usr/local/lib/`) and locally (i.e., :code:`~/opt/lib`), 
and you want to use the local one. Then, you can use :code:`find_library` with the :code:`NO_DEFAULT_PATH` flag.


.. literalinclude:: ../../../cmake_tutorial_workspace/src/cpp_cmake_example_qpoases_lib/examples/CMakeLists.txt
    :language: cmake
    :linenos:
    :emphasize-lines: 3,4,9   

