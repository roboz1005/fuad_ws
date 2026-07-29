"""
FILE LOCATION: fuad_ws/src/integration_pkg/setup.py
Registers all Python nodes as console scripts.
"""
from setuptools import find_packages, setup

package_name = 'integration_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/all.launch.py',
            'launch/swarm.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fuad',
    maintainer_email='fuad@example.com',
    description='Integration package for TurtleBot3 + Quadcopter in Gazebo Harmonic',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'follow_node      = integration_pkg.follow_node:main',
            'tf_follow_node   = integration_pkg.tf_follow_node:main',
            'voice_node       = integration_pkg.voice_node:main',
            'selector_node    = integration_pkg.selector_node:main',
            'dashboard_node   = integration_pkg.dashboard_node:main',
            'teleop_bridge    = integration_pkg.teleop_bridge:main',
            'battery_sim      = integration_pkg.battery_sim_node:main',
            'tf_broadcaster   = integration_pkg.tf_broadcaster_node:main',
        ],
    },
)
