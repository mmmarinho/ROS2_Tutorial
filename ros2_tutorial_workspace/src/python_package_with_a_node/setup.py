from setuptools import find_packages, setup

package_name = 'python_package_with_a_node'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample_python_node = python_package_with_a_node.sample_python_node:main',
            'print_forever_node = python_package_with_a_node.print_forever_node:main'
        ],
    },
)
