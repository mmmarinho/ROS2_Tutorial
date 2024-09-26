.. include:: ../../the_section_is_optional.rst

.. _Editing Python source:

Editing Python source (with :program:`PyCharm`)
===============================================

There are near-infinite ways to manage your Python code and, in this section, we will use :program:`PyCharm`. Namely, the free community version.

Installing :program:`PyCharm`
----------------------------

:program:`PyCharm` is a great program for managing one's Python sources that is frequently updated and has a free edition. However, precisely because it is frequently updated, there is no way for this tutorial to keep up with future changes.

What we will do, instead, is to download a specific version of :program:`PyCharm` for these tutorials, so that its behavior/looks/menus are predictable. If you'd prefer using the shiniest new version, be sure to wear sunglasses and not stare directly into the light.

Run

.. code :: console

   cd ~
   mkdir ros2_workspace_pycharm
   cd ros2_workspace_pycharm
   wget https://download.jetbrains.com/python/pycharm-community-2023.1.1.tar.gz
   tar -xzvf pycharm-community-2023.1.1.tar.gz
   
Create an alias for :program:`pycharm_ros2`
-------------------------------------------

.. note:
   Starting :program:`PyCharm` from the terminal has the added benefit of easily recognizing our ROS2, as long as it has been started from a properly sourced terminal. 

To simplify the use of this version of :program:`PyCharm`, let us create a :program:`bash` alias for it. 

.. code :: console

   echo "# Alias for PyCharm, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
   echo "alias pycharm_ros2=~/ros2_workspace_pycharm/pycharm-community-2023.1.1/bin/pycharm.sh" >> ~/.bashrc
   source ~/.bashrc
   
Then, you can run :program:`PyCharm` with

.. code :: console

    pycharm_ros2

.. info::
   We use :program:`pycharm_ros2` instead of :program:`pycharm` on purpose. This prevents our alias from misbehaving if you have another version of :program:`PyCharm` installed.

2024.09 Update
--------------

.. note:
   This is a troubleshooting section. You do not need to cover this if you don't have problems with the previous section.

For consistency, until I upgrade this tutorial to the new version of ROS2, the version of :program:`pycharm` will remain the same. I have identified the following problem when running it on an `arm64` Ubuntu VM. 

.. code :: console

   CompileCommand: exclude com/intellij/openapi/vfs/impl/FilePartNodeRoot.trieDescend
   Error occurred during initialization of VM
   java.lang.UnsupportedClassVersionError: com/intellij/util/lang/PathClassLoader has been compiled by a more recent version of the Java Runtime (class file version 61.0), this version of the Java Runtime only recognizes class file versions up to 55.0
   	at java.lang.ClassLoader.defineClass1(java.base@11.0.24/Native Method)
   	at java.lang.ClassLoader.defineClass(java.base@11.0.24/ClassLoader.java:1022)
   	at java.security.SecureClassLoader.defineClass(java.base@11.0.24/SecureClassLoader.java:174)
   	at jdk.internal.loader.BuiltinClassLoader.defineClass(java.base@11.0.24/BuiltinClassLoader.java:800)
   	at jdk.internal.loader.BuiltinClassLoader.findClassOnClassPathOrNull(java.base@11.0.24/BuiltinClassLoader.java:698)
   	at jdk.internal.loader.BuiltinClassLoader.loadClassOrNull(java.base@11.0.24/BuiltinClassLoader.java:621)
   	at jdk.internal.loader.BuiltinClassLoader.loadClass(java.base@11.0.24/BuiltinClassLoader.java:579)
   	at jdk.internal.loader.ClassLoaders$AppClassLoader.loadClass(java.base@11.0.24/ClassLoaders.java:178)
   	at java.lang.ClassLoader.loadClass(java.base@11.0.24/ClassLoader.java:527)
   	at java.lang.Class.forName0(java.base@11.0.24/Native Method)
   	at java.lang.Class.forName(java.base@11.0.24/Class.java:398)
   	at java.lang.ClassLoader.initSystemClassLoader(java.base@11.0.24/ClassLoader.java:1981)
   	at java.lang.System.initPhase3(java.base@11.0.24/System.java:2091)

My java version before updating is

.. code :: console

   openjdk version "11.0.24" 2024-07-16
   OpenJDK Runtime Environment (build 11.0.24+8-post-Ubuntu-1ubuntu322.04)
   OpenJDK 64-Bit Server VM (build 11.0.24+8-post-Ubuntu-1ubuntu322.04, mixed mode)


This can be solved by adding `java 17`, because that is what the `class file version 61.0` stands for. To install `java 17` we can run

.. code :: console

   sudo apt install openjdk-17-jre

