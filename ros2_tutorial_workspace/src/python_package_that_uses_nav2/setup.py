from setuptools import find_packages, setup

package_name = 'python_package_that_uses_nav2'

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
    maintainer='root',
    maintainer_email='murilo.marinho@manchester.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav2_initial_pose_publisher_node = python_package_that_uses_nav2.nav2_initial_pose_publisher_node:main',
            'nav2_navigate_to_pose_action_client_node = python_package_that_uses_nav2.nav2_navigate_to_pose_action_client_node:main'
        ],
    },
)
