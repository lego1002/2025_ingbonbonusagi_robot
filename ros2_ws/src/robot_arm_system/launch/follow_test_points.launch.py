from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # robot_arm_system view rviz
    robot_pkg = get_package_share_directory('robot_arm_system')
    view_launch = os.path.join(robot_pkg, 'launch', 'view_robot.launch.py')

    # ev3 bridge launch
    bridge_pkg = get_package_share_directory('ev3_mqtt_bridge')
    bridge_launch = os.path.join(bridge_pkg, 'launch', 'ros_mqtt_bridge_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(view_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),

        # 路徑執行器（讀 JSON → IK → action）
        Node(
            package='robot_arm_system',
            executable='path_ik_executor',
            name='path_ik_executor',
            output='screen',
            parameters=[{
                'json_path': os.path.expanduser('~/Desktop/2025_ingbonbonusagi_robot/ros2_ws/test_point.json'),
                'use_interpolation': True,
                'interp_step_m': 0.01,
                'xyz_scale': 1.0,  # 你的 JSON 是公尺就 1.0；如果是公分就 0.01
            }]
        ),
    ])

