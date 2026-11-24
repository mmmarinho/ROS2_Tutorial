from setuptools import find_packages, setup

package_name = 'python_package_with_shutdown_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='murilo',
    maintainer_email='murilomarinho@ieee.org',
    description='Nodes that differ only in how main() ends, with their measured Ctrl-C behaviour',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'good_template = python_package_with_shutdown_demos.good_template:main',
            'bad_bare_shutdown = python_package_with_shutdown_demos.bad_bare_shutdown:main',
            'bad_uncaught_external_shutdown = python_package_with_shutdown_demos.bad_uncaught_external_shutdown:main',
            'warn_ok_guard = python_package_with_shutdown_demos.warn_ok_guard:main',
            'warn_keyboard_interrupt_only = python_package_with_shutdown_demos.warn_keyboard_interrupt_only:main',
            'warn_try_shutdown = python_package_with_shutdown_demos.warn_try_shutdown:main',
            'warn_try_shutdown_only = python_package_with_shutdown_demos.warn_try_shutdown_only:main',
        ],
    },
)
