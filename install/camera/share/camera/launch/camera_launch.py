from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam_params = {
        'device': '/dev/video2',
        'width': 1920,
        'height': 1080,
        'fps': 30,
        'pixel_format': 'MJPG',

        'exposure_mode_manual': True,
        'exposure_time_abs': 156,
        'dynamic_fps': False,
        'awb_auto': False,
        'af_auto': False,
        'buffersize': 1,
        
        'show_window': True,    
        'topic': 'camera/image_raw',
        'window_name': 'Camera V4L2 (Auto Config)',
    }
    
    return LaunchDescription([
        # active camera
        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[cam_params],
        ),
        
        # undistort
        # Node(
        #     package='camera',
        #     executable='undistort',
        #     name='undistort',
        #     output='screen'
        # ),
        
        # # noise_filtered
        # Node(
        #     package='camera',
        #     executable='noise',
        #     name='noise',
        #     output='screen'
        # ),
        
        # # edge
        Node(
            package='camera',
            executable='sobel',
            name='sobel',
            output='screen'
        ),
        
        # # fine line
        # Node(
        #     package='camera',
        #     executable='findLine',
        #     name='findLine',
        #     output='screen'
        # ),
        
        
    ])
