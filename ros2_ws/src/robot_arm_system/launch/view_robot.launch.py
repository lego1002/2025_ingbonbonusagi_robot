import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# [修正] 應該是 launch.substitutions (中間是點，不是底線)
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robot_arm_system'
    pkg_share = get_package_share_directory(package_name)

    # 1. 取得 URDF 路徑
    default_model_path = os.path.join(pkg_share, 'urdf/lego_arm.urdf')

    # 2. 取得 Rviz Config 路徑
    rviz_config_path = os.path.join(pkg_share, 'rviz/lego_arm.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', default_model_path])}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])