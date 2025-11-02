Representing Frame Transformations
==================================

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
    
where :math:`||\boldsymbol{v}||=1`.
  
The easiest way to think about a rotation using quaternions is to think about the axis of rotation :math:`\boldsymbol{v}` and the angle of rotation :math:`\phi`.
Then, you construct the quaternion with the :ref:`rotation quaternion formation law <eq:rotation_formation>`.

.. admonition:: Examples

    Suppose that we have a rotation of :math:`\phi=\pi` radians (angles always in radians, don't forget!).

    If we want such a rotation about the x-axis, we choose :math:`\boldsymbol{v}_1=\hat{\imath}`. Therefore, this rotation would be correctly represented by

    .. math::

        \boldsymbol{r}_1 \triangleq \cos\left(\frac{\pi}{2}\right) + \hat{\imath}\sin\left(\frac{\pi}{2}\right).

    If we want such a rotation about the z-axis, we choose :math:`\boldsymbol{v}_2=\hat{k}`, correctly represented by

    .. math::

        \boldsymbol{r}_2 \triangleq \cos\left(\frac{\pi}{2}\right) + \hat{k}\sin\left(\frac{\pi}{2}\right).

    Any :math:`\boldsymbol{v}` is acceptable as long as the norm is one. For instance, :math:`\boldsymbol{v}_3=-\sqrt{2}\hat{\imath} + \sqrt{2}\hat{k}`
    leads to the valid rotation quaternion

    .. math::

         \boldsymbol{r}_3 \triangleq \cos\left(\frac{\pi}{2}\right) + \left(-\sqrt{2}\hat{\imath} + \sqrt{2}\hat{k}\right)\sin\left(\frac{\pi}{2}\right).


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

    TODO

Similarly, for a ``rotation``, we have the following.

- ``x`` will store the value of the term related to :math:`\hat{\imath}` in :math:`\boldsymbol{r}`.
- ``y`` will store the value of the term is related to :math:`\hat{\jmath}` in :math:`\boldsymbol{r}`.
- ``z`` will store the value of the term is related to :math:`\hat{k}` in :math:`\boldsymbol{r}`.

Lastly, we have, for a ``rotation``,

- ``w`` will store the value of the term not related to any imaginary unit.

.. admonition:: Example

    To represent the following elementary rotation about the x-axis

    .. math::

        \boldsymbol{r}_1 \triangleq \cos\left(\frac{\pi}{2}\right) + \hat{\imath}\sin\left(\frac{\pi}{2}\right),


    we would assign ``w = cos(pi/2)``, ``x = sin(pi/2)``, ``y = 0``, and ``z = 0`` in our program.



