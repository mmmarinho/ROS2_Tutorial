"""
This level of indirection prevents us from getting a `RuntimeWarning` like the one below.

RuntimeWarning: 'minimalist_package.test_minimalist_class' found in sys.modules after import of package
'minimalist_package', but prior to execution of 'minimalist_package.test_minimalist_class'; this may result in
unpredictable behaviour
  warn(RuntimeWarning(msg))
"""

from ._test_minimalist_class import main

main()
