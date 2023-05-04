Create packages (:program:`ros2 pkg create`)
---------------

:code:`ROS2` has a tool to help creating package templates. We can get all available options by running

.. code-block:: console
   
   ros2 pkg create -h

which outputs a list of handy options to populate the package template with useful files. Namely, the four emphasized ones.

.. code-block:: console
   :emphasize-lines: 27, 29, 35, 37

    usage: ros2 pkg create [-h] [--package-format {2,3}] [--description DESCRIPTION]
                           [--license LICENSE]
                           [--destination-directory DESTINATION_DIRECTORY]
                           [--build-type {cmake,ament_cmake,ament_python}]
                           [--dependencies DEPENDENCIES [DEPENDENCIES ...]]
                           [--maintainer-email MAINTAINER_EMAIL]
                           [--maintainer-name MAINTAINER_NAME] [--node-name NODE_NAME]
                           [--library-name LIBRARY_NAME]
                           package_name

    Create a new ROS 2 package

    positional arguments:
      package_name          The package name

    options:
      -h, --help            show this help message and exit
      --package-format {2,3}, --package_format {2,3}
                            The package.xml format.
      --description DESCRIPTION
                            The description given in the package.xml
      --license LICENSE     The license attached to this package; this can be an arbitrary
                            string, but a LICENSE file will only be generated if it is one
                            of the supported licenses (pass '?' to get a list)
      --destination-directory DESTINATION_DIRECTORY
                            Directory where to create the package directory
      --build-type {cmake,ament_cmake,ament_python}
                            The build type to process the package with
      --dependencies DEPENDENCIES [DEPENDENCIES ...]
                            list of dependencies
      --maintainer-email MAINTAINER_EMAIL
                            email address of the maintainer of this package
      --maintainer-name MAINTAINER_NAME
                            name of the maintainer of this package
      --node-name NODE_NAME
                            name of the empty executable
      --library-name LIBRARY_NAME
                            name of the empty library
