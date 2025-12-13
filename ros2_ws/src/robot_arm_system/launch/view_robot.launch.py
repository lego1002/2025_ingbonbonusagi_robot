import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robot_arm_system'
    pkg_share = get_package_share_directory(package_name)

    # 1. 取得 URDF 路徑
    default_model_path = os.path.join(pkg_share, 'urdf/lego_arm.urdf')
    # 2. 取得 Rviz Config 路徑
    rviz_config_path = os.path.join(pkg_share, 'rviz/lego_arm.rviz')

    # 3. 機器人狀態發布節點 (TF 生成器)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['cat ', default_model_path])}]
    )

    # 4. [新增] 關節狀態發布節點 (代理人)
    # 功能：沒收到指令時發布 0 度；收到 /ik_joint_states 時轉發該角度
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/ik_joint_states'],  # 監聽我們 Python 程式發出的 Topic
            'robot_description': Command(['cat ', default_model_path])
        }]
    )

    # 5. Rviz 節點
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,  # 記得把新節點加進來
        rviz_node
    ])