"""
It is common practice to name internal modules with a trailing '_', so that
they can be correctly imported in the '__init__.py'.
"""

"""
MIT LICENSE

Copyright (C) 2023 Murilo Marques Marinho (www.murilomarinho.info)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


class MinimalistClass:
    """
    A minimalist class example with the most used elements.
    https://docs.python.org/3/tutorial/classes.html
    """
    # Attribute reference, accessed with MinimalistClass.attribute_reference
    attribute_reference: str = "Hello "

    def __init__(self,
                 attribute_arg: float = 10.0,
                 private_attribute_arg: float = 20.0):  # With a default value of 20.0
        """The __init__ works together with __new__ (not shown here) to
        construct a class. Loosely it is called the Python 'constructor' in
        some references, although it is officially an 'initializer' hence
        the name.
        https://docs.python.org/3/reference/datamodel.html#object.__init__
        It customizes an instance with input arguments.
        """
        # Attribute that can be accessed externally
        self.attribute: float = attribute_arg

        # Attribute that should not be accessed externally
        # a name prefixed with an underscore (e.g. _spam) should be treated
        # as a non-public part of the API (whether it is a function, a method or a data member).
        # It should be considered an implementation detail and subject to change without notice.
        self._private_attribute: float = private_attribute_arg

    def method(self) -> float:
        """Methods with 'self' should use at least one statement in which 'self' is required."""
        return self.attribute + self._private_attribute

    def set_private_attribute(self, private_attribute_arg: float) -> None:
        """If a private attribute should be writeable, define a setter."""
        self._private_attribute = private_attribute_arg

    def get_private_attribute(self) -> float:
        """If a private attribute should be readable, define a getter."""
        return self._private_attribute

    @staticmethod
    def static_method():
        """
        Methods that do not use the 'self' should be decorated with the @staticmethod.
        It will only have access to attribute references.
        https://docs.python.org/3.10/library/functions.html#staticmethod
        """
        return MinimalistClass.attribute_reference + "World!"
