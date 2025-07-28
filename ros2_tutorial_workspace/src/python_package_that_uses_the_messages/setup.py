from setuptools import find_packages, setup

package_name = 'python_package_that_uses_the_messages'

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
            'amazing_quote_publisher_node = python_package_that_uses_the_messages.amazing_quote_publisher_node:main',
            'amazing_quote_subscriber_node = python_package_that_uses_the_messages.amazing_quote_subscriber_node:main'
        ],
    },
)
