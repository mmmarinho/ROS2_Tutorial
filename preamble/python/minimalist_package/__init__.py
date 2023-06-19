"""
Having an __init__.py file within a directory turns it into a Python Package.
A package within a package is called a subpackage.
https://docs.python.org/3/tutorial/modules.html#packages
"""
from ._minimalist_class import MinimalistClass
from . import test_minimalist_class
