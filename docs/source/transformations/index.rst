Frame Transformations
=====================

.. include:: ../the_topic_is_under_heavy_construction.rst

Unless made of flexible materials, the robots we use and the objects they interact with can be well simplified as `rigid bodies <https://en.wikipedia.org/wiki/Rigid_body>`_.

Terminology dictates that a *position* and an *orientation* are inherent elements of rigid bodies, related to information about their instantaneous reference frames.
In this sense, a *rotation* and a *translation* would represent the position and the orientation of a rigid body with respect to a reference frame and that implies dislocation.

However, mathematically, positions and orientations are *always* represented with respect to a reference frame.
I believe that, because of this, beyond this tutorial, in robotics and computer science, you will often see position/translation
and orientation/rotation used interchangeably.

Please allow me to use only the terms *translation* and *rotation* henceforth to keep the discussion consistent
with how it is utilised in :program:`ROS2`.

We start by looking at how it is done in :program:`ROS2`, go through the related mathematics, then create an example
to show how to create these transformations.

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
    
where :math:`\boldsymbol{v}^2=-1`. This means that the rotation axis :math:`\boldsymbol{v}` can be any imaginary number
such that :math:`||\boldsymbol{v}||=1`.
  
The easiest way to think about a rotation using quaternions is to think about the axis of rotation :math:`\boldsymbol{v}` and the angle of rotation :math:`\phi`.
Then, you construct the quaternion with the :ref:`rotation quaternion formation law <eq:rotation_formation>`.

.. admonition:: Examples

    We can choose :math:`\phi=0` and see how a quaternion represents no rotation. regardless of rotation axis, this results in

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

    Any :math:`\phi \in \mathbb{R}` is acceptable and, more importantly, any :math:`\boldsymbol{v}` is acceptable as long as it is imaginary and has norm one.
    For instance, :math:`\boldsymbol{v}_3=-\sqrt{2}\hat{\imath} + \sqrt{2}\hat{k}` leads to the valid rotation quaternion

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

Sequential rotations can be obtained via quaternion multiplication. For instance, to obtain the result of a rotation
of :math:`\phi=\pi` about the x-axis followed by a rotation of the same angle about the z-axis, we do

.. math::

    \boldsymbol{r}_{12} \triangleq \boldsymbol{r}_1\boldsymbol{r}_2,

which results, in this case,

.. math::

    \boldsymbol{r}_{12} &= \left[\cos\left(\frac{\pi}{2}\right) + \hat{\imath}\sin\left(\frac{\pi}{2}\right)\right]\left[\cos\left(\frac{\pi}{2}\right) + \hat{k}\sin\left(\frac{\pi}{2}\right)\right] \\
                        &= \hat{\imath}\hat{k} \\
                        &= -\hat{\jmath}.

The product of any two quaternions can always be done algebraically, by respecting the `multiplication between the basis elements <https://en.wikipedia.org/wiki/Quaternion#Multiplication_of_basis_elements>`_.
The result can be generalised by the `quaternion multiplication <https://en.wikipedia.org/wiki/Quaternion#Hamilton_product>`_.
This generalization will not be repeated here as we will do it programmatically later.

Inverse rotations
+++++++++++++++++

The inverse rotation can be obtained by flipping the axis of rotation. This is equivalent to obtaining the so-called quaternion
conjugate of unit-norm quaternions.

.. math::
    :name: eq:rotation_inverse

    \boldsymbol{r}^{*} \triangleq \cos\left(\frac{\phi}{2}\right) - \boldsymbol{v}\sin\left(\frac{\phi}{2}\right).

We can see that this is indeed the inverse rotation by noticing that the rotation multiplied by its conjugate results
in :math:`1`, that is, the non rotation.

.. math::

    \boldsymbol{r}\boldsymbol{r}^{*} = \boldsymbol{r}^{*}\boldsymbol{r} &= \left[\cos\left(\frac{\phi}{2}\right) - \boldsymbol{v}\sin\left(\frac{\phi}{2}\right)\right]\left[\cos\left(\frac{\phi}{2}\right) + \boldsymbol{v}\sin\left(\frac{\phi}{2}\right)\right] \\
                                     &= \cos^2\left(\frac{\phi}{2}\right) - \boldsymbol{v}\boldsymbol{v}\sin^2\left(\frac{\phi}{2}\right) \\
                                     &= \cos^2\left(\frac{\phi}{2}\right) - (-1)\sin^2\left(\frac{\phi}{2}\right) \\
                                     &= \cos^2\left(\frac{\phi}{2}\right) + \sin^2\left(\frac{\phi}{2}\right) \\
                                     &= 1.

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

Create the package
------------------

We will create a package to showcase the transformations from the previous section. We use ``TransformedStamped`` as
it will be useful right away, when we talk about ``tf2``.

To see how this would work, programmatically, we start by creating the :file:`python_package_that_uses_geometry_msgs` package.
Note that it must depend on ``geometry_msgs``.

.. code-block:: console

    cd ~/ros2_tutorial_workspace/src
    ros2 pkg create python_package_that_uses_geometry_msgs \
    --build-type ament_python \
    --dependencies geometry_msgs

.. dropdown:: ros2 pkg create output

    .. code-block:: console

        going to create a new package
        package name: python_package_that_uses_geometry_msgs
        destination directory: ~/ros2_tutorial_workspace/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['root <murilo.marinho@manchester.ac.uk>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: ['geometry_msgs']
        creating folder ./python_package_that_uses_geometry_msgs
        creating ./python_package_that_uses_geometry_msgs/package.xml
        creating source folder
        creating folder ./python_package_that_uses_geometry_msgs/python_package_that_uses_geometry_msgs
        creating ./python_package_that_uses_geometry_msgs/setup.py
        creating ./python_package_that_uses_geometry_msgs/setup.cfg
        creating folder ./python_package_that_uses_geometry_msgs/resource
        creating ./python_package_that_uses_geometry_msgs/resource/python_package_that_uses_geometry_msgs
        creating ./python_package_that_uses_geometry_msgs/python_package_that_uses_geometry_msgs/__init__.py
        creating folder ./python_package_that_uses_geometry_msgs/test
        creating ./python_package_that_uses_geometry_msgs/test/test_copyright.py
        creating ./python_package_that_uses_geometry_msgs/test/test_flake8.py
        creating ./python_package_that_uses_geometry_msgs/test/test_pep257.py

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

Highlighted below are the files that we will create or modify.

.. code-block:: console
    :emphasize-lines: 5,9

    python_package_that_uses_geometry_msgs/
    |-- package.xml
    |-- python_package_that_uses_geometry_msgs
    |   |-- __init__.py
    |   `-- create_stamped_transforms_node.py
    |-- resource
    |   `-- python_package_that_uses_geometry_msgs
    |-- setup.cfg
    |-- setup.py
    `-- test
        |-- test_copyright.py
        |-- test_flake8.py
        `-- test_pep257.py

Add sample code
---------------

Create the following sample Python script. It will serve to show the operations we did earlier mathematically.

:download:`create_stamped_transforms_node.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_geometry_msgs/python_package_that_uses_geometry_msgs/create_stamped_transforms_node.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_geometry_msgs/python_package_that_uses_geometry_msgs/create_stamped_transforms_node.py
   :language: python
   :lines: 24-
   :linenos:

Note that the only novelty will be the excerpt below. We create an instance of ``TransformStamped`` and start by
adding information related to the header. Each transform has a frame of reference, called ``header.frame_id``. Then,
the actual frame this transform represents is held by ``child_frame_id``.

The values corresponding to example translation :math:`\boldsymbol{t}_1 \triangleq 1\hat{\imath} + 2\hat{\jmath} + 3\hat{k}` and the
example rotation :math:`\boldsymbol{r}_1 \triangleq \cos\left(\frac{\pi}{2}\right) + \hat{\imath}\sin\left(\frac{\pi}{2}\right)`
are assigned in the highlight lines below.

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_geometry_msgs/python_package_that_uses_geometry_msgs/create_stamped_transforms_node.py
   :language: python
   :lines: 36-60
   :emphasize-lines: 15,16,17,20,21,22,23

Update the :file:`setup.py`
---------------------------

As usual, we add the necessary entry point in :file:`setup.py`.

:download:`setup.py <../../../ros2_tutorial_workspace/src/python_package_that_uses_geometry_msgs/setup.py>`

.. literalinclude:: ../../../ros2_tutorial_workspace/src/python_package_that_uses_geometry_msgs/setup.py
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

    ros2 run python_package_that_uses_geometry_msgs create_stamped_transforms_node

The result will be as follows.

.. code-block:: console

    This transform has translation: geometry_msgs.msg.Vector3(x=1.0, y=2.0, z=3.0) and rotation: geometry_msgs.msg.Quaternion(x=1.0, y=0.0, z=0.0, w=6.123233995736766e-17).

Note that the results are reasonably close to the ones we calculated mathematically.
However, given the limitations on current computers related to floating point accuracy, you can always expect a level of inaccuracy.
This is not limited or affected by the use of quaternions, this is an inherent limitation of our computers.
