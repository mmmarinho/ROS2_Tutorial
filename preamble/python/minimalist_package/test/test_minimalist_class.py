import unittest
from minimalist_package import MinimalistClass


class TestMinimalistClass(unittest.TestCase):
    """For each `TestCase`, we create a subclass of `unittest.TestCase`."""

    def setUp(self):
        self.minimalist_instance = MinimalistClass(attribute_arg=15.0,
                                                   private_attribute_arg=35.0)

    def test_attribute(self):
        self.assertEqual(self.minimalist_instance.attribute, 15.0)

    def test_private_attribute(self):
        self.assertEqual(self.minimalist_instance._private_attribute, 35.0)

    def test_method(self):
        self.assertEqual(self.minimalist_instance.method(), 15.0 + 35.0)

    def test_get_set_private_attribute(self):
        self.minimalist_instance.set_private_attribute(20.0)
        self.assertEqual(self.minimalist_instance.get_private_attribute(), 20.0)

    def test_static_method(self):
        self.assertEqual(MinimalistClass.static_method(), "Hello World!")


def main():
    unittest.main()
