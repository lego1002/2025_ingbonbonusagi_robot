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

        # ★ 必須加入這一行，ROS 才找得到 Launch 檔
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # ★★★ 關鍵！必須加入這一行，ROS 才找得到 URDF 檔 ★★★
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # ★ 加入這一行，讓 install 資料夾裡也有 rviz 設定檔
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
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
            'ik_test = robot_arm_system.ik_test:main',
            'ik_test_lego_using = robot_arm_system.ik_test_lego_using:main',
            'path_ik_executor = robot_arm_system.path_ik_executor:main',
            'zero_manager = robot_arm_system.zero_manager:main',
        ],
    },
)
