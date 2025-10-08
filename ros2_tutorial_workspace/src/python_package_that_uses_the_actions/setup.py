from setuptools import find_packages, setup

package_name = 'python_package_that_uses_the_actions'

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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_straight_in_2d_action_server_node = python_package_that_uses_the_actions.move_straight_in_2d_action_server_node:main',
            'move_straight_in_2d_action_client_node = python_package_that_uses_the_actions.move_straight_in_2d_action_client_node:main'
        ],
    },
)
