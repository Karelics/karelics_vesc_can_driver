#!/usr/bin/env python
# -*- coding: utf-8 -*-
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['karelics_vesc_can_driver'],
    package_dir={'': 'include'})

setup(**setup_args)


from setuptools import setup
package_name = 'karelics_vesc_can_driver'
setup(
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mart Moerdijk',
    maintainer_email='mart.moerdijk@karelics.fi',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)