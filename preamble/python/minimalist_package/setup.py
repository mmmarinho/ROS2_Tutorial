from setuptools import setup, find_packages

package_name = 'minimalist_package'

setup(
    name=package_name,
    version='23.6.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Murilo M. Marinho',
    maintainer_email='murilomarinho@ieee.org',
    description='A minimalist package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'minimalist_script = minimalist_package.minimalist_script:main',
            'async_await_example = minimalist_package.minimalist_async.async_await_example:main',
            'async_callback_example = minimalist_package.minimalist_async.async_callback_example:main'
        ],
    },
)
