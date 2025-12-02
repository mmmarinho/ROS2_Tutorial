Cybersecurity
=============

.. include:: ../the_topic_is_under_heavy_construction.rst

Let's start with a quick definition and a main reference for cybersecurity terminology.

   "Cybersecurity is the protection of information assets from harm." :footcite:p:`Josang2024`

Anything related to robotics is based on my own experience, contextualised with the terminology of references.

Robotic systems are particularly susceptible to cybersecurity issues. In my experience, they tend to have relatively
many *vulnerabilities*. For instance, robotic systems

- frequently have access to the internet through institutional/company networks. This is an increasing vulnerability given the popularity of cloud-based LLMs.
- might have an outdated mainstream operating system. Robotic systems might run older Linux variants and embedded Windows systems. The onboard software is rarely ever updated. Sometimes it can't be updated owing to specific device drivers.
- frequently are not (or cannot) managed by the central IT infrastructure as this could reduce their effectiveness or make them unpractical. Examples are firewalls and anti-viruses.
- could have manufacture's software with unauditable behaviour the is infrequently (or never) patched or documented.

This means that a *threat actor* can find many *attack vectors* to interfere with robotics infrastructure. The usual
*impacts* we usually think of in terms of cybersecurity might be loss of data or data leakage. In a robotics context,
loss of data could be the loss of photos of power lines stored in a vulnerable inspection drone. Data leakage might be,
for example, videos of your house streamed via your cleaning robot or inspection videos taken by the latest robotic dog of a
classified facility.

One additional impact of compromised robotic systems can be physical harm. Past are the days in which software was the
thing you cursed and hardware the thing you punched. Now hardware can punch right back. Or even unprovoked.

Cybersecurity in robotic systems are (or should be) a real, and increasing, *risk*. Differently from a WiFi-connected refrigerator that
might be more a display of wealth than actual need, our dear robots tend to benefit from (or need) some level of connectivity.

Basic terminology
-----------------

The terminology can be slightly different between sources, see for instance :footcite:p:`Josang2024` and :footcite:p:`Santos2022`.
Behold my incredible storytelling skills materialised into my own amalgamation among existing terminology.

Think of your robot's computer system as your house. The one in which you have a `precious ring <https://www.youtube.com/watch?v=Iz-8CSa9xj8>`_. In this context,
each of the cyber-security terminology becomes as follows.

- Attacker: the bad person wearing a balaclava that wants to steal your precious.
- Vulnerability: issues with the security. For instance, leaving the front-door key under a potted plant.
- Attack vector: the way in which a vulnerability is exploited by a threat actor to steal your precious. For instance, the balaclava-person lifting up the potted plant, picking up the front-door key that was under the potted plant, and proceeding to open the door with it.
- Impact: the problem you'll have if you're attacked. In this case, your precious will be stolen. You'll cry. Your contents insurance payments will rise.
- Risk: impact put into perspective of likelihood. How many balaclava-clad people want your precious, really? Alternatively, being a neighbor of `Sauron <https://en.wikipedia.org/wiki/Sauron>`_ might indicate high risk of losing your precious.


What should I do?
-----------------

The backbone of everything we will discuss in this topic will be *encryption*, part of cryptography which is very
much literally "the science of secret writing" :footcite:p:`Josang2024`. Other common-sense IT security actions also apply for
robotic systems. These include backing things up, using firewalls, having strong passwords, keeping systems updated, not having things
connected to the internet when they don't have to be, trusting noone, never storing personal data on a robot, and so on.

Scope of this tutorial
----------------------

Among cybersecurity controls, two will stand out in this tutorial. Mainly network isolation and encryption, which are types of preventive
controls, and backup, which is a type of corrective control.

.. admonition:: References

    .. footbibliography::