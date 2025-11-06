:program:`rviz2` for ``tf2``
============================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. hint::

    An official :program:`rviz2` user guide is available at:

    - https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html.

:program:`rviz2` Installation
-----------------------------

:program:`rviz2` has already been installed in :ref:`ROS2 installation`.

What is :program:`rviz2`?
-------------------------

Possibly the easiest description for :program:`rviz2`, available from https://github.com/ros2/rviz, is *ROS 3D Robot Visualizer*.

Given that :program:`rviz2` lacks any integration with simulation engines, it wouldn't be usually called a robotics simulator.
Nonetheless, for most robotics applications in their initial stages of development, a visualization is an important step forward.

Using :program:`rviz2` with ``tf2``
-----------------------------------

One interesting capability of :program:`rviz2` is the visualization of ``tf2`` in a relatively friendly manner.
We can test that quickly with the ``tf2`` example we developed in the previous section.

Up until now, I showered you with equations that you might or might not have understood. After showing you how it works in :program:`rviz2`, hopefully
you will `begin to believe <https://youtu.be/aXNLdw9CUgE?si=E0bfNXsXn3afTLsz>`_ me.

.. tab-set::

    .. tab-item:: Terminal 1: tf2 broadcaster

        From our previous example of ``tf2``.

        .. code-block:: console

            ros2 run python_package_that_uses_tf2 tf2_broadcaster_node

    .. tab-item:: Terminal 2: rviz2

        We run :program:`rviz2` with the following command.

        .. code-block:: console

            ros2 run rviz2 rviz2


The user interface of :program:`rviz2` should appear. However, it still does not know what you are trying to do with it, so we
will make two modifications to be able to visualize the output of our :file:`tf2_broadcaster_node`.

#. In :menuselection:`Displays --> Fixed Frame`, we change the text from *map* to *world*.
#. Then, we add the ``tf2`` visualisation with the sequence  :menuselection:`Add --> rviz_default_plugins --> TF`.

This is summarised in the video below.

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/jMxvI_EdlIc?si=a3cGn8wnjIHe8-fp" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

This should be enough to start showing the frames. Using the mouse, can you zoom in and rotate the window to focus
on the transforms of interest.

If everything worked as expected, you should see a ``robot_1`` frame rotating about a ``world`` frame.
Now, you can easily visualize the frames indicating that the frame marked by ``robot_1`` indeed moves in circles, with
the rotational frame also rotating in the same frequency.

With these settings, additional broadcast transforms will also be visible. Using :program:`rviz2` menus, you can filter
the transforms that are relevant for you at any given point.

Other visualisation tools
-------------------------

Depending on the sensorial information, :program:`rviz2` can be the correct tool. It is important, however, to know
that it is not the only tool. One convenient tool for visualizing images is :program:`rqt_image_view`.

The integrated sample can be executed as follows.

.. danger::

    Please note that the (Lena or Lenna) image has been banned by most publishers. Do not use it in your
    reports or conference submissions. This is part of the integrated sample, therefore shown here, but this usage is not endorsed or recommended.

    - https://www.nature.com/articles/s41565-018-0337-2
    - https://journals.ieeeauthorcenter.ieee.org/create-your-ieee-journal-article/create-graphics-for-your-article/

    Nature:

        We would like to let our authors, reviewers and readers know that, with immediate effect, we no longer consider submissions containing the Lena (sometimes ‘Lenna’) image. This decision was taken in consultation with relevant journal editors and affects all Nature Research journals.

    IEEE:

        Lena Image

        IEEE’s diversity statement and supporting policies such as the IEEE Code of Ethics speak to IEEE’s commitment to
        promoting an inclusive and equitable culture that welcomes all.  In alignment with this culture and with respect
        to the wishes of the subject of the image, Lena Forsén, IEEE will no longer accept submitted papers which include the “Lena image.”


.. tab-set::

    .. tab-item:: Terminal 1: Publish sample images.

        .. code-block:: console

            ros2 run rqt_image_view image_publisher

    .. tab-item:: Terminal 2: Run the bridge

        .. code-block:: console

            ros2 run rqt_image_view rqt_image_view /images

It should work by showing color variations of the controversial image. We will use the visualiser, not the publisher,
in some examples in future sections.