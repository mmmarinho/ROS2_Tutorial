Public-key cryptosystems
========================

.. include:: ../the_topic_is_under_heavy_construction.rst

.. note::

    This is obviously a simplified discussion of the topic.

ROS2 Security
+++++++++++++

.. seealso::

    Official information: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Security.html

There are facilities in :program:`ROS2` to enable secure communication. The communication uses cryptography which
will be shown in the following section. It is important to know that such capability exists. It will have a number
of steps needed to create the necessary certificates to guarantee security among nodes.

After that is set up, nodes and program:`ros2cli` tools can be called with additional security.

.. seealso::

    Official information https://docs.ros.org/en/jazzy/Tutorials/Advanced/Security/Introducing-ros2-security.html

Altering these settings can have unwanted side effects to other parts of the tutorial so we will leave this topic
only briefly mentioned. In addition, understanding these topics requires first understanding public-key cryptography, shown below.

Scope of this section
+++++++++++++++++++++

Most of the important infrastructure for software for robotics relies heavily the concept of public-key cryptosystems.
Many attribute this concept to as early as `1977 <https://patents.google.com/patent/US4405829>`_, in one type of encryption
algorithm (`RSA <https://en.wikipedia.org/wiki/RSA_cryptosystem>`_) that is still in use.

Understanding the general idea of public-key encryption is rather simple. We do not need to enter the mathematical details.

Assumptions
+++++++++++

- Every bit of every message that anyone sends over the internet can be seen by any number of people and eavesdropped at all times.
    - This is very much the case. Whatever we do is visible by the ISP and anyone in the route it takes from you and the server.
- Only trust the identity of someone you can verify.
    - IP addresses are dynamic, `MAC addresses <https://en.wikipedia.org/wiki/MAC_address>`_ can be easily spoofed, and so on.

What does this mean, in practice? This means that using unencrypted messages someone can easily steal your information
through the internet. For instance, suppose that you have to send your password to your bank. If this goes through a
large number of unknown computers until reaching your bank's server (which might be anywhere, really), everyone will
know that your password is ``Ilov3mon3y£££``.

Creating keys
+++++++++++++

We are going to use the ``ssh-keygen`` tool to illustrate how this works in practice. This will also be useful for you
when working with robots at least in two ways. First, when you need to connect remotely to a robot using ``ssh``. Second,
when you need to upload your version controlled-data, e.g. fancy backup, in remote servers such as GitHub.

.. code-block:: console

    ssh-keygen --help

This program is used to create the *encryption keys* used in public-key encryption. You will see few algorithms available,
namely ``dsa``, ``ecdsa``, ``ecdsa-sk``, ``ed25519``, ``ed25519-sk``, ``rsa``. Please feel free to look them up to see the differences.

.. code-block:: console
    :emphasize-lines: 4

    unknown option -- -
    usage: ssh-keygen [-q] [-a rounds] [-b bits] [-C comment] [-f output_keyfile]
                      [-m format] [-N new_passphrase] [-O option]
                      [-t dsa | ecdsa | ecdsa-sk | ed25519 | ed25519-sk | rsa]
                      [-w provider] [-Z cipher]
           ssh-keygen -p [-a rounds] [-f keyfile] [-m format] [-N new_passphrase]
                       [-P old_passphrase] [-Z cipher]
           ssh-keygen -i [-f input_keyfile] [-m key_format]
           ssh-keygen -e [-f input_keyfile] [-m key_format]
           ssh-keygen -y [-f input_keyfile]
           ssh-keygen -c [-a rounds] [-C comment] [-f keyfile] [-P passphrase]
           ssh-keygen -l [-v] [-E fingerprint_hash] [-f input_keyfile]
           ssh-keygen -B [-f input_keyfile]
           ssh-keygen -D pkcs11
           ssh-keygen -F hostname [-lv] [-f known_hosts_file]
           ssh-keygen -H [-f known_hosts_file]
           ssh-keygen -K [-a rounds] [-w provider]
           ssh-keygen -R hostname [-f known_hosts_file]
           ssh-keygen -r hostname [-g] [-f input_keyfile]
           ssh-keygen -M generate [-O option] output_file
           ssh-keygen -M screen [-f input_file] [-O option] output_file
           ssh-keygen -I certificate_identity -s ca_key [-hU] [-D pkcs11_provider]
                      [-n principals] [-O option] [-V validity_interval]
                      [-z serial_number] file ...
           ssh-keygen -L [-f input_keyfile]
           ssh-keygen -A [-a rounds] [-f prefix_path]
           ssh-keygen -k -f krl_file [-u] [-s ca_public] [-z version_number]
                      file ...
           ssh-keygen -Q [-l] -f krl_file [file ...]
           ssh-keygen -Y find-principals -s signature_file -f allowed_signers_file
           ssh-keygen -Y match-principals -I signer_identity -f allowed_signers_file
           ssh-keygen -Y check-novalidate -n namespace -s signature_file
           ssh-keygen -Y sign -f key_file -n namespace file [-O option] ...
           ssh-keygen -Y verify -f allowed_signers_file -I signer_identity
                      -n namespace -s signature_file [-r krl_file] [-O option]

We are going to go mostly with the defaults. However, we need to be sure to create the keys in another file so that
it does not create problems with your own system. We will create the keys with the following command.

.. code-block:: console

    ssh-keygen -f example_ed25519

Press :kbd:`Enter` a couple of times to create a key without a passphrase. The passphrase will be useful if someone
might have access to your computer but isn't too poorly intended. It's a thin layer of extra safety. We can skip it
for the purposes of this tutorial.

.. code-block:: console

    Generating public/private ed25519 key pair.
    Enter passphrase (empty for no passphrase):
    Enter same passphrase again:
    Your identification has been saved in example_ed25519
    Your public key has been saved in example_ed25519.pub
    The key fingerprint is:
    SHA256:Hb2mpOg1Ya5NUPXzl8JVFzmDdsRtj6JcECsIqnIN/EU root@c869fbce1a11
    The key's randomart image is:
    +--[ED25519 256]--+
    |    . E   o.  +o*|
    | . . o . ..+ o *=|
    |  +   o o o.= .++|
    | . + . . o .++o o|
    |o . o . S.ooo+...|
    |..     = +oo  .. |
    |      . * .      |
    |     . = .       |
    |      o .        |
    +----[SHA256]-----+

.. danger::

    I will show the keys and share the results here, because this is obviously a tutorial. However, you should always
    be careful with your keys. If your private key, explained below, is shared, it is compromised. This means you will
    have to create new keys and re-add them to any relevant systems before someones steals your precious.

In the same folder in which the command was executed, we will see two files ``example_ed25519.pub`` and ``example_ed25519``.
The first one, with the extension ``.pub`` is the **public** key. It is a string of pretty much random characters. The
one I just created is shown below.

.. code-block:: console

    cat example_ed25519.pub

The command above will output the public key.

.. code-block:: console

    ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIAU1eT9YRbosGzdgxbN38pb+jH3nwXfKwtJPCbQgLY3n root@c869fbce1a11

We can also see the contents of the private key, similarly. Let's keep the suspense for now, to stay in the proper
narrative line.

How do we send a secret message?
++++++++++++++++++++++++++++++++

Let us start by installing a supporting program, called :program:`age`. This program will allow us to use our encryption
keys to encrypt specific messages. Usually, :program:`ssh` would handle those more complex connections for us. We are
only using :program:`age` to illustrate the ideas behind public-key cryptography.

.. code-block:: console

    sudo apt-get update
    sudo apt-get install age

Suppose that you have a very important text to encrypt and send to your best friend. However, you might be slightly `tsundere <https://en.wikipedia.org/wiki/Tsundere>`_
about it and don't want anyone else to see it.

Let's create the secret message, called :file:`secret_message.txt`.

.. code-block:: console

    echo "This is the best tutorial I have every seen thanks Murilo for being so great." > secret_message.txt

Now, the contents are obviously visible to anyone. You want to be sure that only your friend can see it. Therefore, you
grab your friend's public key. Let's use :file:`example_ed25519.pub` I showed as an example for this. Again, the public
key is not a secret, it is meant to be *public*, that is, seen by other people. People, then, only need to be able to
trust that you are the holder of this specific key pair.

Let's use our cool program, :program:`age`, that allows us to play with encryption. Without further ado, let's go with it.

.. code-block:: console

    age -R example_ed25519.pub secret_message.txt > secret_message.txt.age

Now, the message has been encrypted. We can see the contents of :file:`secret_message.txt.age` as follows. This is what
you will send your friend. You can even copy and paste it on something fully public, like an Instagram post. Your other
friends might be concerned seeing that but otherwise your message is still a secret.

.. code-block:: console

    age-encryption.org/v1
    -> ssh-ed25519 Hb2mpA xxu02M6ZYcgdvpjQ0OtObLX67P+C1cr/6AefZ+w/g1I
    R3mtqZ9x8sz54/j8g2qY/2EJvkQytXZnLOPmwTziY+w
    --- GiV9DHB2gAr4V6ZFhIMPH81sDEEfjCGocYImCYD/lhA
    1�
      �,l�j
           ݚ��&
               �CF��gf��V�Y*ez����5
                                   �bu�ང>ǒ ��n=˨g�}M]��     �e�#����^Zmuh���jj�o���ȅ�l_�root@c869fbce1a11

The magic is that nobody will be able to decrypt the message unless the have the *private* key. Let's summarise
what just happened.

- Anyone who wants to participate on an encrypted conversation make their *public* encryption key available.
- Anyone who wants to send them a secret message uses that public key to encrypt the message.
- Only the person with the *private* can decrypt the message.

Therefore, they are able to send secret messages through a compromised and public channel, e.g. the internet. As long as
the public key is correct, we do not have to trust the identity of anyone either. **ONLY** someone who holds the private
key will be able to read the true contents of the original message.

How do we read a secret message?
++++++++++++++++++++++++++++++++

So, suppose that you sent the secret message, :file:`secret_message.txt.age`, above to your friend. Your friend will now have to use their private
key to decrypt the message. Again, suppose that the keys we just created are safely held by your friend. If they
want to read the contents of the secret message they will have to use the matching private key.

.. code-block:: console

    cat example_ed25519

The output is somewhat similar to the public key. I will not show it here to reinforce that it is something you should
keep private. Your friend will use the private key :file:`example_ed25519` and the secret file they just received
from you :file:`secret_message.txt.age`.

.. code-block:: console

    age -d -i example_ed25519 secret_message.txt.age > secret_message_decrypted.txt

Then, you can confirm that it has been decrypted with the following command.

.. code-block:: console

    cat secret_message_decrypted.txt

In which the original message is restored.

.. code-block:: console

    This is the best tutorial I have every seen thanks Murilo for being so great.

You can confirm that you won't be able to decrypt anything that was
encrypted with the example public key. That is because I haven't showed you the private key. I'm pretty sure
I lost it too.

.. danger::

    If you lose your private key, any information you had only in encrypted form is lost forever. FOREVER.

Wait, what?
+++++++++++

Yes, the magic happens because of the key pair. Each key, alone, is weak (actually, meaningless). `Keys together, strong <https://www.youtube.com/watch?v=20LuSlZT4S4>`_.

So, for an encrypted conversation between two participants, there will be two key pairs. One for each participant.
Suppose that participant A has public key A and private key A. Then, suppose that participant B has public key B and
private key B.

The flow in this case would be as follows.

- Participant A will use public key B to encrypt a message.
- It will be sent to participant B.
- Participant B will decrypt the message with private key B.
- Participant B will use public key A to encrypt a response.
- It will be sent to participant A.
- Participant A will decrypt the message with private key A.

Because the public keys can be freely seen through a public channel, e.g., the internet, the information exchanged is
safe. This does not mean that encryption is not crackable. With enough time and opportunities to attack, a private key
can theoretically be eventually guessed. This is to loosely one of the ideas behind `cryptocurrencies <https://en.wikipedia.org/wiki/Cryptocurrency>`_,
in which a `hash <https://en.wikipedia.org/wiki/Cryptographic_hash_function>`_ must be guessed.

Exercises
+++++++++

We can think of decryption and encryption exercises, that help illustrate the process.

Decryption
~~~~~~~~~~

Suppose that you receive the following message, which has been encrypted with your public key. It was clearly done so
using :program:`age`.

.. code-block:: console

    age-encryption.org/v1
    -> ssh-ed25519 AMMzdA /ODwrsfFiJeEtlH2EuPzo8aENrDgsfb7gkdEcDMc3VA
    GGajPsTFCAREmweT3f43+8cUJ1H+FQj/Oiv2uTcv/Ts
    --- nnYExEUF5LrEfYqwXvLzGcr1eNcYPr3nuipLfflevSM

This is the pairing private key that you have in your computer, which you should never ever share with anyone for any
reason. Anyone with this key can decode the message.

.. caution::

    I will show the private key here because this is a tutorial. DO NOT SHARE YOUR PRIVATE KEY WITH ANYONE.

.. code-block:: console

    -----BEGIN OPENSSH PRIVATE KEY-----
    b3BlbnNzaC1rZXktdjEAAAAABG5vbmUAAAAEbm9uZQAAAAAAAAABAAAAMwAAAAtzc2gtZW
    QyNTUxOQAAACCmUlsdqJr1dJUzSX2rSctLWrifN3FtXw0bhd+ACRet/QAAAKDv9N2x7/Td
    sQAAAAtzc2gtZWQyNTUxOQAAACCmUlsdqJr1dJUzSX2rSctLWrifN3FtXw0bhd+ACRet/Q
    AAAEDmqGfF7PfgEOBtbzsuZqocWgSAAmX4+zqMmhZZ+NBZDKZSWx2omvV0lTNJfatJy0ta
    uJ83cW1fDRuF34AJF639AAAAHWI0MDYxN21tQMOvwr/CvcOvwr/CvcOvwr/CvWVV
    -----END OPENSSH PRIVATE KEY-----

What are the decrypted contents of this message?

Encryption
~~~~~~~~~~

Suppose that you have the following public key.

.. code-block:: console

    ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIKZSWx2omvV0lTNJfatJy0tauJ83cW1fDRuF34AJF639 b40617mm@ï¿½ï¿½ï¿½eU

Encrypt the message below. Well, anything will do, really.

    "Sorry kid, you got the gift but it looks like you are waiting for something, next life maybe who knows".

.. dropdown:: Private key for you to test

    Use this private key to test if your encryption was correct or not.

    .. code-block:: console
    
        -----BEGIN OPENSSH PRIVATE KEY-----
        b3BlbnNzaC1rZXktdjEAAAAABG5vbmUAAAAEbm9uZQAAAAAAAAABAAAAMwAAAAtzc2gtZW
        QyNTUxOQAAACCmUlsdqJr1dJUzSX2rSctLWrifN3FtXw0bhd+ACRet/QAAAKDv9N2x7/Td
        sQAAAAtzc2gtZWQyNTUxOQAAACCmUlsdqJr1dJUzSX2rSctLWrifN3FtXw0bhd+ACRet/Q
        AAAEDmqGfF7PfgEOBtbzsuZqocWgSAAmX4+zqMmhZZ+NBZDKZSWx2omvV0lTNJfatJy0ta
        uJ83cW1fDRuF34AJF639AAAAHWI0MDYxN21tQMOvwr/CvcOvwr/CvcOvwr/CvWVV
        -----END OPENSSH PRIVATE KEY-----

