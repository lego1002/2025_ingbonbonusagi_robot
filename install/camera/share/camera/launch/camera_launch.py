from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # active camera
        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        
        # Node(
        #     package='camera',
        #     executable='noise',
        #     name='noise',
        #     output='screen'
        # ),
        
        #undistort
        Node(
            package='camera',
            executable='undistort',
            name='undistort',
            output='screen'
        ),
        
        # edge
        Node(
            package='camera',
            executable='sobel',
            name='sobel',
            output='screen'
        ),
        
        # Node(
        #     package='camera',
        #     executable='canny',
        #     name='canny',
        #     output='screen'
        # ),
        
        # fine line
        Node(
            package='camera',
            executable='findLine',
            name='findLine',
            output='screen'
        ),
        
        
    ])
