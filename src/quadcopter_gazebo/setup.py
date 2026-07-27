from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadcopter_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models/quadcopter', glob('models/quadcopter/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fuad',
    maintainer_email='fuad@example.com',
    description='Minimal quadcopter model for Gazebo Harmonic + ROS 2 Jazzy',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'teleop_to_motor = quadcopter_gazebo.teleop_to_motor_node:main',
            'motor_mixer     = quadcopter_gazebo.motor_mixer_node:main',
        ],
    },
)
