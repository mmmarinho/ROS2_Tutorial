import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'python_package_that_uses_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config_bridge'), glob(os.path.join('config_bridge', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='murilo.marinho@manchester.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "send_wrenches_to_gazebo_node = python_package_that_uses_gazebo.send_wrenches_to_gazebo_node:main",
            "send_poses_to_gazebo_node = python_package_that_uses_gazebo.send_poses_to_gazebo_node:main",
            "control_shape_thrust_node = python_package_that_uses_gazebo.control_shape_thrust_node:main"
        ],
    },
)
