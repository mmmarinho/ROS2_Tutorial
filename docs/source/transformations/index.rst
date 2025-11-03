Frame Transformations
=====================

.. include:: ../the_topic_is_under_heavy_construction.rst

Unless made of flexible materials, the robots we use and the objects they interact with can be well simplified as `rigid bodies <https://en.wikipedia.org/wiki/Rigid_body>`_.

Terminology dictates that a *position* and an *orientation* are inherent elements of rigid bodies, related to information about their instantaneous reference frames.
In this sense, a *rotation* and a *translation* would represent the position and the orientation of a rigid body with respect to a reference frame and that implies dislocation.

However, mathematically, every position and orientation is *always* represented with respect to a reference frame.
I believe that, because of this, in robotics and computer science, you will often see position/translation
and orientation/rotation used interchangeably.
Please allow me to use only the terms *translation* and *rotation* henceforth to keep the discussion consistent
with how it is utilised in :program:`ROS2`.

Translations
------------

Translations :math:`\boldsymbol{t} \in \mathbb{R}^3` can be easily represented using imaginary numbers, such as

.. math::
    :name: eq:translation_formation

    \boldsymbol{t} \triangleq t_x\hat{\imath} + t_y\hat{\jmath} + t_z\hat{k},


where :math:`\hat{\imath}^2 = \hat{\jmath}^2 = \hat{k}^2 = \hat{\imath} \hat{\jmath} \hat{k} = -1`.

If you thought this would never be useful, think again. It's about to be!

Rotations: Quaternions
----------------------

.. note::

    I'm not explaining this (in a failed attempt) to show off. :program:`ROS2` represents rotations as quaternions,
    so this knowledge is needed in the short term as well.

Whereas translations are intuitive and can be easily processed with A-levels mathematics, rotations demand extra thought.
However, the complexity is exaggerated to some extent.

In :program:`ROS2`, rotations are represented as quaternions.
They have several benefits over other representations, which you can see in related literature on quaternions.
For robotics, it is easier to think of rotations directly in quaternions instead of relying on intermediary ways, such as Euler angles.
The reason is that the particularities of quaternions will eventually catch up to you, no matter how long and how far one might try to run away from it.

Let's rip the trademarked plaster brand out. The following equation represents the formation of a rotation quaternion:

.. math::
    :name: eq:rotation_formation

    \boldsymbol{r} \triangleq \cos\left(\frac{\phi}{2}\right) + \boldsymbol{v}\sin\left(\frac{\phi}{2}\right),
    
where :math:`\boldsymbol{v}\boldsymbol{v}=-1`. The condition of the rotation axis also implies that :math:`||\boldsymbol{v}||=1`.
  
The easiest way to think about a rotation using quaternions is to think about the axis of rotation :math:`\boldsymbol{v}` and the angle of rotation :math:`\phi`.
Then, you construct the quaternion with the :ref:`rotation quaternion formation law <eq:rotation_formation>`.

.. admonition:: Examples

    We can choose :math:`\phi=0` and see how what quaternion represents no rotation. For any rotation axis, this results in

        .. math::

            \boldsymbol{r}_0 &\triangleq \cos\left(\frac{0}{2}\right) + \boldsymbol{v}\sin\left(\frac{0}{2}\right) \\
                             &= 1.

    Now, suppose that we have a rotation of :math:`\phi=\pi` radians (angles always in radians, don't forget!).

    If we want such a rotation about the x-axis, we choose :math:`\boldsymbol{v}_1=\hat{\imath}`. Therefore, this rotation would be correctly represented by

        .. math::

            \boldsymbol{r}_1 &\triangleq \cos\left(\frac{\pi}{2}\right) + \hat{\imath}\sin\left(\frac{\pi}{2}\right) \\
                             &= \hat{\imath}.

    If we want such a rotation about the z-axis, we choose :math:`\boldsymbol{v}_2=\hat{k}`, correctly represented by

        .. math::

            \boldsymbol{r}_2 &\triangleq \cos\left(\frac{\pi}{2}\right) + \hat{k}\sin\left(\frac{\pi}{2}\right) \\
                             &= \hat{k}.

    Any :math:`\phi \in \mathbb{R}` is acceptable and, more importantly, any :math:`\boldsymbol{v}` is acceptable as long as the norm is one. For instance, :math:`\boldsymbol{v}_3=-\sqrt{2}\hat{\imath} + \sqrt{2}\hat{k}`
    leads to the valid rotation quaternion

        .. math::

             \boldsymbol{r}_3 &\triangleq \cos\left(\frac{\pi}{2}\right) + \left(-\sqrt{2}\hat{\imath} + \sqrt{2}\hat{k}\right)\sin\left(\frac{\pi}{2}\right) \\
                              &= \left(-\sqrt{2}\hat{\imath} + \sqrt{2}\hat{k}\right).


.. error::

    There are also common pitfalls.

    #. Using a rotation axis that is not unit norm does **not** represent a rotation quaternion.

        Using :math:`\boldsymbol{v}_4=-\hat{\imath} + \hat{k}` is **not** a valid rotation quaternion because its norm is not one, :math:`||\boldsymbol{v}_4||=\sqrt{2}`.

    #. Misunderstanding the rotation angle.

        Note that, inside the quaternion, we have :math:`\frac{\phi}{2}`. For instance, the quaternion

        .. math::

            \boldsymbol{r}_5 \triangleq \cos\left(\frac{\pi}{4}\right) + \hat{\jmath}\sin\left(\frac{\pi}{4}\right),

        represents a rotation of :math:`\frac{\pi}{2}` about the y-axis, **not** :math:`\frac{\pi}{4}`.


Sequential rotations
++++++++++++++++++++

Sequential rotations can be obtained via the quaternion multiplication. For instance, to obtain the result of a rotation
of :math:`\phi=\pi` about the x-axis followed by a rotation of the same angle about the z-axis, we do

.. math::

    \boldsymbol{r}_{12} \triangleq \boldsymbol{r}_1\boldsymbol{r}_2

which becomes

.. math::

    \boldsymbol{r}_{12} &= \left[\cos\left(\frac{\pi}{2}\right) + \hat{\imath}\sin\left(\frac{\pi}{2}\right)\right]\left[\cos\left(\frac{\pi}{2}\right) + \hat{k}\sin\left(\frac{\pi}{2}\right)\right] \\
                        &= \hat{\imath}\hat{k} \\
                        &= -\hat{\jmath}

Inverse rotations
+++++++++++++++++

The inverse rotation can be obtained by flipping the axis of rotation. This is equivalent to obtaining the so-called quaternion
conjugate.

.. math::
    :name: eq:rotation_inverse

    \boldsymbol{r}^{*} \triangleq \cos\left(\frac{\phi}{2}\right) - \boldsymbol{v}\sin\left(\frac{\phi}{2}\right),

We can see that this is indeed the rotation by noticing that

.. math::

    \boldsymbol{r}\boldsymbol{r}^{*} = \boldsymbol{r}^{*}\boldsymbol{r} &= \left[\cos\left(\frac{\phi}{2}\right) - \boldsymbol{v}\sin\left(\frac{\phi}{2}\right)\right]\left[\cos\left(\frac{\phi}{2}\right) + \boldsymbol{v}\sin\left(\frac{\phi}{2}\right)\right] \\
                                     &= \cos^2\left(\frac{\phi}{2}\right) - \boldsymbol{v}\boldsymbol{v}\sin^2\left(\frac{\phi}{2}\right) \\
                                     &= \cos^2\left(\frac{\phi}{2}\right) - (-1)\sin^2\left(\frac{\phi}{2}\right) \\
                                     &= \cos^2\left(\frac{\phi}{2}\right) + \sin^2\left(\frac{\phi}{2}\right) \\
                                     &= 1.


Transformations in :program:`ROS2`
----------------------------------

Let us take a look at message most commonly used in :program:`ROS2` to represent translation and rotation,
:file:`TransformStamped.msg`, part of the package :file:`geometry_msgs`.

We can see its contents with

.. code-block:: console

    ros2 interface show geometry_msgs/msg/TransformStamped

resulting in a slightly intricate message.

.. dropdown:: ros2 interface show output

    .. code-block:: yaml

        # This expresses a transform from coordinate frame header.frame_id
        # to the coordinate frame child_frame_id at the time of header.stamp
        #
        # This message is mostly used by the
        # <a href="https://docs.ros.org/en/rolling/p/tf2/">tf2</a> package.
        # See its documentation for more information.
        #
        # The child_frame_id is necessary in addition to the frame_id
        # in the Header to communicate the full reference for the transform
        # in a self contained message.

        # The frame id in the header is used as the reference frame of this transform.
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id

        # The frame id of the child frame to which this transform points.
        string child_frame_id

        # Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
        Transform transform
                Vector3 translation
                        float64 x
                        float64 y
                        float64 z
                Quaternion rotation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1

To simplify the discussion, these are the contents of :file:`TransformStamped.msg`.

.. rli:: https://raw.githubusercontent.com/ros2/common_interfaces/refs/heads/jazzy/geometry_msgs/msg/TransformStamped.msg
   :language: yaml

Note that it is a message composed of two other messages, and a built-in.

The :file:`Header` is an essential timestamp used throughout :program:`ROS2`. We can see its contents with

.. code-block:: console

    ros2 interface show std_msgs/msg/Header

resulting in

.. code-block:: yaml

    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data
    # in a particular coordinate frame.

    # Two-integer timestamp that is expressed as seconds and nanoseconds.
    builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec

    # Transform frame with which this data is associated.
    string frame_id

The elements :code:`stamp` and :code:`frame_id` are used by many popular packages :program:`ROS2` to display and
relate translational and rotational data of robots and objects.

Then, to the most important part of this section, we have the :file:`Transform.msg`, whose contents can be obtained
with

.. code-block:: console

    ros2 interface show geometry_msgs/msg/Transform

resulting in

.. code-block:: yaml

    # This represents the transform between two coordinate frames in free space.

    Vector3 translation
            float64 x
            float64 y
            float64 z
    Quaternion rotation
            float64 x 0
            float64 y 0
            float64 z 0
            float64 w 1

in which the :code:`translation` and :code:`rotation` can be clearly seen.

Connecting these ideas
----------------------

For the purposes of this section, we want to connect a :code:`translation` (in the code) to a :ref:`translation<eq:translation_formation>` (from the equation)
and a :code:`rotation` (in the code) to a :ref:`rotation <eq:rotation_formation>` (from the equation).

For a ``translation``, we have the following.

- ``x`` will store the value of the term related to :math:`\hat{\imath}` in :math:`\boldsymbol{t}`, that is :math:`t_x`.
- ``y`` will store the value of the term is related to :math:`\hat{\jmath}` in :math:`\boldsymbol{t}`, that is :math:`t_y`.
- ``z`` will store the value of the term is related to :math:`\hat{k}` in :math:`\boldsymbol{t}`, that is :math:`t_z`.

.. admonition:: Example

    To represent the following translation,

    .. math::

        \boldsymbol{t}_1 \triangleq 1\hat{\imath} + 2\hat{\jmath} + 3\hat{k},

    we would assign ``translation.x = 1``, ``translation.y = 2``, and ``translation.z = 3`` in our program.

Similarly, for a ``rotation``, we have the following.

- ``x`` will store the value of the term related to :math:`\hat{\imath}` in :math:`\boldsymbol{r}`.
- ``y`` will store the value of the term is related to :math:`\hat{\jmath}` in :math:`\boldsymbol{r}`.
- ``z`` will store the value of the term is related to :math:`\hat{k}` in :math:`\boldsymbol{r}`.

Lastly, we have, for a ``rotation``,

- ``w`` will store the value of the term not related to any imaginary unit.

.. note::

    You might be wondering why ``w``, when the other dimensions might be already natural to you.
    We need to give it a name in our programs so that we can store its data.
    In alphabetical order, ``w`` comes before the letters we usually use for the x, y, and z dimensions.

.. admonition:: Example

    To represent the following elementary rotation about the x-axis

    .. math::

        \boldsymbol{r}_1 \triangleq \cos\left(\frac{\pi}{2}\right) + \hat{\imath}\sin\left(\frac{\pi}{2}\right),


    we would assign ``rotation.w = cos(pi/2)``, ``rotation.x = sin(pi/2)``, ``rotation.y = 0``, and ``rotation.z = 0`` in our program.

Installing a support library ``nottf2``
---------------------------------------

.. note::

    You might be wondering.

        Why do I need this?

    When using ``tf2`` there are only two complexities.

    #. Handling ``std_msgs/msg/Header``
    #. Handling ``geometry_msgs/msg/Quaternion``

    Handling headers can be done rather easily. Dealing with quaternions can be more complicated if a support library
    is not used to keep the code clean.

    If you're interested, here's the quaternion explanation available in the official documentation instead.

    -  https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html.


In ``rclpy``, ``tf2`` does not (currently?) have quaternion operations. But I got you covered. Kinda. It's more use at your
own risk type of arrangement.

.. caution::

    The ``nottf2`` library is provided as-is, with no warranty. Use at your own risk (See ``MIT`` license for more details).

    The creator of ``nottf2``, me, will not accept any blame or liability for any issues, including but limited to broken
    robots or broken code in your assignments. If you feel it's insufficient or unreliable, please create and use your own
    implementation, or someone else's that fits your needs.


For the library to be properly found in our environment, it needs to be installed in the system's Python.
We can do so as follows.

.. code-block:: console

    python3 -m pip install nottf2 --break-system-packages


Create :program:`ros2` package that uses ``nottf2``
---------------------------------------------------

Let's first create our sample package, as follows, that will depend on ``geometry_msgs``.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_nottf2 \
    --build-type ament_python \
    --dependencies geometry_msgs

We will be presented with the usual output.

.. dropdown:: ros2 pkg create output

    .. code-block:: console

        going to create a new package
        package name: python_package_that_uses_nottf2
        destination directory: ~/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: []
        creating folder ./python_package_that_uses_nottf2
        creating ./python_package_that_uses_nottf2/package.xml
        creating source folder
        creating folder ./python_package_that_uses_nottf2/python_package_that_uses_nottf2
        creating ./python_package_that_uses_nottf2/setup.py
        creating ./python_package_that_uses_nottf2/setup.cfg
        creating folder ./python_package_that_uses_nottf2/resource
        creating ./python_package_that_uses_nottf2/resource/python_package_that_uses_nottf2
        creating ./python_package_that_uses_nottf2/python_package_that_uses_nottf2/__init__.py
        creating folder ./python_package_that_uses_nottf2/test
        creating ./python_package_that_uses_nottf2/test/test_copyright.py
        creating ./python_package_that_uses_nottf2/test/test_flake8.py
        creating ./python_package_that_uses_nottf2/test/test_pep257.py

        [WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
        It is recommended to use one of the ament license identifiers:
        Apache-2.0
        BSL-1.0
        BSD-2.0
        BSD-2-Clause
        BSD-3-Clause
        GPL-3.0-only
        LGPL-3.0-only
        MIT
        MIT-0

Package structure
-----------------

Below are the files that we will create or modify.

.. code-block:: console
    :emphasize-lines: 5, 9

    python_package_that_uses_nottf2/
    |-- package.xml
    |-- python_package_that_uses_nottf2
    |   |-- __init__.py
    |   `-- operations_showcase.py
    |-- resource
    |   `-- python_package_that_uses_nottf2
    |-- setup.cfg
    |-- setup.py
    `-- test
        |-- test_copyright.py
        |-- test_flake8.py
        `-- test_pep257.py

Add sample code for ``nottf2``
------------------------------

.. danger::

    You do not need this support library if you're just creating basic rotations. It becomes useful if you need to,
    for instance, multiply quaternions to obtain relative rotations with ``quaternion_multiply``.

Create the following sample Python script. It will serve to show the operations we did earlier mathematically.

:download:`operations_showcase.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_nottf2/python_package_that_uses_nottf2/operations_showcase.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nottf2/python_package_that_uses_nottf2/operations_showcase.py
   :language: python
   :lines: 24-
   :linenos:

Update the :file:`setup.py`
---------------------------

As usual, we add the necessary entry point in :file:`setup.py`. We also add ``nottf2`` as an usual Python dependency, although,
in :program:`colcon`, this currently seems to be mostly cosmetic.

:download:`setup.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_nottf2/setup.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_nottf2/setup.py
   :language: python
   :emphasize-lines: 15,24


Build and source
----------------

Before we proceed, let us build and source once.

.. include:: ../the_canonical_build_command.rst

Run Example
-----------

We run our newly created program as follows.

.. code-block:: console

    ros2 run python_package_that_uses_nottf2 operations_showcase

The result will be as follows.

.. code-block:: console

    The rotation of 3.141592653589793 radians about the x-axis is r1=geometry_msgs.msg.Quaternion(x=1.0, y=0, z=0, w=6.123233995736766e-17).
    The inverse rotation of r1 is r1_conj=geometry_msgs.msg.Quaternion(x=-1.0, y=0, z=0, w=6.123233995736766e-17).
    The rotation of 3.141592653589793 radians about the z-axis is r2=geometry_msgs.msg.Quaternion(x=0, y=0, z=1.0, w=6.123233995736766e-17).
    The quaternion multiplication of r12=r1r2 is r12=geometry_msgs.msg.Quaternion(x=6.123233995736766e-17, y=-1.0, z=6.123233995736766e-17, w=3.749399456654644e-33).

Note that the results are reasonably close to the ones we calculated mathematically. However, given the limitations
on current computers related to floating point accuracy, you can **always** expect a level of inaccuracy. This is **not**
limited or affected by the use of quaternions, this is an inherent limitation of our computers.
