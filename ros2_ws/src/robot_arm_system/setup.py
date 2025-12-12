from setuptools import setup
import os
from glob import glob

package_name = 'robot_arm_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Required for ROS2 to find your package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='you@example.com',
    description='Camera + IK + distributor system for LEGO robot arm',
    license='MIT',
    tests_require=['pytest'],

    # REGISTER ALL YOUR NODES HERE
    entry_points={
        'console_scripts': [
            'camera_node = robot_arm_system.camera_node:main',
            'fixed_target = robot_arm_system.fixed_target:main',
            'ik_node = robot_arm_system.ik_node:main',
            'distributor = robot_arm_system.distributor_node:main',
        ],
    },
)
