Representing rotations
======================

.. include:: ../the_topic_is_under_heavy_construction.rst


As we have seen in the actions section of the tutorial, pose messages are composed of a position part and an orientation part.

Terminology dictates that a *position* and an *orientation* are inherent elements of rigid bodies, related to information about their instantaenous reference frames. In this logic, a *rotation* and a *translation* would represent the position and the orientation of a rigid body with respect to a reference frame and that implies dislocation.

However, mathematically, every position and orientation is represented with respect to a reference frame. I believe that, because of this, in robotics and computer science, you will often see position/translation and orientation/rotation used interchangibly. Please allow me to use only the term *rotation* henceforth to keep the discussion consistent.

Quaternions
-----------

Whereas positions are mathematically intuitive and can be easily processsed with A-levels mathematics, rotations demand extra thought. However, the complexity is misticised to some extent. 


Positions :math:`\boldsymbol{p} \in \mathbb{R}^3` can be easily represented using imaginary numbers, such as

.. math::

    \boldsymbol{p} \triangleq p_x\imath + p_y\jmath + p_z\hat{k}


where :math:`\imath^2 = \jmath^2 = \hat{k}^2 = \imath \jmath \hat{k} = -1`. If you thought this would never be useful, think again. It's about to be!


In :program:`ROS2`, rotations are represented as quaternions. They have several benefits, which you can see in related literature on quaternions. For robotics, it is easier to think of rotations directly in quaternions instead of relying on intermediary ways, such as Euler angles. The reason is that the particularities of quaternions will evantually catch up to you, no matter how long and how far one might try to run away from it. 

Let's rip the trademarked plaster brand out. The following equation represents the formation of a rotation quaternion:

.. math::

    \boldsymbol{r} \triangleq \cos(\frac{\phi}{2}) + \boldsymbol{v}\sin(\frac{\phi}{2}),
    
where :math:`\norm{v}=1`. It reads as follows

.. code:: console

    r defined as cosine of phi over two plus v times sine of phi over two, where norm of v equals one.
    
  




References
----------

I have not been an avid user of :code:`tf` myself. I can somewhat see the appeal at an attempt to abstract away some of the mathematics of pose transformations. 


The resource mentioned below when accessed for the purposes of this tutorial was in an incomplete state.
    An official resource using  :program:`tf2` for quaternions is  https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html.


