from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_robot = get_package_share_directory('robot_arm_system')
    pkg_ev3 = get_package_share_directory('ev3_mqtt_bridge')

    view_launch = os.path.join(pkg_robot, 'launch', 'view_robot.launch.py')
    bridge_launch = os.path.join(pkg_ev3, 'launch', 'ros_mqtt_bridge_launch.py')

    return LaunchDescription([
        # 1) RViz + robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(view_launch)
        ),

        # 2) EV3 MQTT bridge + motor actions
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch)
        ),

        # 3) Zero manager（先歸零）
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robot_arm_system',
                    executable='zero_manager',
                    name='zero_manager',
                    output='screen',
                )
            ]
        ),

        # 4) Path IK executor（直接啟動 Node，不再 include 其他 launch）
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='robot_arm_system',
                    executable='path_ik_executor',
                    name='path_ik_executor',
                    output='screen',
                    parameters=[{
                        'json_path': os.path.expanduser(
                            '~/Desktop/2025_ingbonbonusagi_robot/ros2_ws/test_point.json'
                        ),
                        'use_interpolation': True,
                        'interp_step_m': 0.01,
                        'xyz_scale': 1.0,
                    }]
                )
            ]
        ),
    ])
