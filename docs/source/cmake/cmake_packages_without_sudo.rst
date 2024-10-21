


Install a CMake package without sudo privileges
-----------------------------------------------

Create a folder :program:`opt` in /home/USERNAME/opt

.. code-block:: console

    mkdir opt && cd opt
    mkdir include
    mkdir lib



Update the LD_LIBRARY_PATH in :program:`~/.bashrc`
--------------------------------------------------

.. code-block:: console

    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/USERNAME/opt/lib" >> ~/.bashrc
    source ~/.bashrc


Example:

.. image:: ../../images/cmake_without_sudo_steps.gif
   :align: center    