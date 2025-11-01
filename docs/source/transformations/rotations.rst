Representing rotations
======================

.. include:: ../the_topic_is_under_heavy_construction.rst

Unless made of flexible materials, the robots we use and the objects they interact with can be well simplified as `rigid bodies <https://en.wikipedia.org/wiki/Rigid_body>`_.

Terminology dictates that a *position* and an *orientation* are inherent elements of rigid bodies, related to information about their instantaneous reference frames.
In this sense, a *rotation* and a *translation* would represent the position and the orientation of a rigid body with respect to a reference frame and that implies dislocation.

However, mathematically, every position and orientation is *always* represented with respect to a reference frame.
I believe that, because of this, in robotics and computer science, you will often see position/translation
and orientation/rotation used interchangeably.
Please allow me to use only the terms *translation* and *rotation* henceforth to keep the discussion consistent
with how it is utilised in :program:`ROS2`.

Quaternions
-----------

.. note::

    I'm not explaining this (in a failed attempt) to show off. :program:`ROS2` represents rotations as quaternions,
    so this knowledge is needed in the short term as well.

Whereas translations are intuitive and can be easily processed with A-levels mathematics, rotations demand extra thought.
However, the complexity is exaggerated to some extent.

Translations :math:`\boldsymbol{t} \in \mathbb{R}^3` can be easily represented using imaginary numbers, such as

.. math::

    \boldsymbol{t} \triangleq t_x\hat{\imath} + t_y\hat{\jmath} + t_z\hat{k},


where :math:`\hat{\imath}^2 = \hat{\jmath}^2 = \hat{k}^2 = \hat{\imath} \hat{\jmath} \hat{k} = -1`.
If you thought this would never be useful, think again. It's about to be!


In :program:`ROS2`, rotations are represented as quaternions.
They have several benefits over other representations, which you can see in related literature on quaternions.
For robotics, it is easier to think of rotations directly in quaternions instead of relying on intermediary ways, such as Euler angles.
The reason is that the particularities of quaternions will eventually catch up to you, no matter how long and how far one might try to run away from it.

Let's rip the trademarked plaster brand out. The following equation represents the formation of a rotation quaternion:

.. math::
    :name: eq:rotation_formation

    \boldsymbol{r} \triangleq \cos\left(\frac{\phi}{2}\right) + \boldsymbol{v}\sin\left(\frac{\phi}{2}\right),
    
where :math:`||\boldsymbol{v}||=1`. It reads as follows

.. code:: console

    r defined as cosine of phi over two plus v times sine of phi over two, where norm of v equals one.
    
  
The easiest way to think about a rotation using quaternions is to think about the axis of rotation :math:`\boldsymbol{v}` and the angle of rotation :math:`\phi`.
Then, you construct the quaternion with the :ref:`rotation quaternion formation law <eq:rotation_formation>`.

.. admonition:: Example

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



But bro, I don't care about your maths
--------------------------------------

.. caution::

    Not caring about maths might be reason number one for "unexplainable" issues with robots.

References
----------

I have not been an avid user of :code:`tf` myself. I can somewhat see the appeal at an attempt to abstract away some of the mathematics of pose transformations. 


The resource mentioned below when accessed for the purposes of this tutorial was in an incomplete state.
    An official resource using  :program:`tf2` for quaternions is  https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html.


