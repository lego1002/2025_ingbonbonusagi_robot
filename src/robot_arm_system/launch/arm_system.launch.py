from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='robot_arm_system',
            executable='fixed_target',
            name='fixed_target',
            output='screen',
        ),

        Node(
            package='robot_arm_system',
            executable='ik_node',
            name='ik_node',
            output='screen',
        ),

        Node(
            package='robot_arm_system',
            executable='distributor',
            name='distributor',
            output='screen',
        ),

        # Camera (optional)
        # Node(
        #     package='robot_arm_system',
        #     executable='camera_node',
        #     name='camera',
        #     output='screen',
        # ),
    ])

