import os
from glob import glob
from setuptools import setup

package_name = 'python_package_that_uses_parameters_and_launch_files'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='murilo',
    maintainer_email='murilomarinho@ieee.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amazing_quote_configurable_publisher_node = '
            'python_package_that_uses_parameters_and_launch_files.amazing_quote_configurable_publisher_node:main',
        ],
    },
)
