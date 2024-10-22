

Install a CMake package without sudo privileges
===============================================

When we install a package or library using CMake, their files are usually copied to folders that only sudo users have access (e.g., :code:`/opt`).
Therefore, to install them without sudo privileges we define a custom directory in :code:`/home/USERNAME/`.

In this tutorial, we are going to create and use a custom folder :code:`opt` in :code:`/home/USERNAME/` containing 
the folders :code:`lib` and :code:`include`.

.. code-block:: console

    mkdir opt && cd opt
    mkdir include
    mkdir lib

Then, we update the LD_LIBRARY_PATH in :code:`~/.bashrc`.    

.. code-block:: console

    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/USERNAME/opt/lib" >> ~/.bashrc
    source ~/.bashrc

Example:

.. image:: ../../images/cmake_without_sudo_steps.gif
   :align: center    


To install a CMake package, we set the :code:`CMAKE_INSTALL_PREFIX:PATH` flag with our custom folder (:code:`/home/USERNAME/opt`)

.. code-block:: console

    cmake -DCMAKE_INSTALL_PREFIX:PATH=/home/USERNAME/opt .. 
    make -j16
    make install


