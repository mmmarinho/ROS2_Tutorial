

Install a CMake package without sudo privileges
===============================================

To install a CMake package or library without sudo privileges, we need to define a directory to which we have access. For instance, 
in :code:`/home/USERNAME/`.


Create a custom folder
----------------------

In this tutorial, we are going to create and use a custom folder :code:`opt` in :code:`/home/USERNAME/` containing 
the folders :code:`lib` and :code:`include`.

.. code-block:: console

    cd ~/
    mkdir opt && cd opt
    mkdir include
    mkdir lib

Then, we update the LD_LIBRARY_PATH in :code:`~/.bashrc`.    

.. code-block:: console

    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/USERNAME/opt/lib" >> ~/.bashrc
    source ~/.bashrc


.. image:: ../../images/cmake_without_sudo_steps.gif
   :align: center    


Install a CMake package
-----------------------

To install a CMake package, we set the :code:`CMAKE_INSTALL_PREFIX:PATH` flag with our custom folder (:code:`/home/USERNAME/opt`)


.. code-block:: console

    cmake -DCMAKE_INSTALL_PREFIX:PATH=/home/USERNAME/opt .. 
    make -j16
    make install


