from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam_params = {
        'device': '/dev/video2',
        'width': 1920,
        'height': 1080,
        'fps': 30,
        'pixel_format': 'MJPG',

        'exposure_mode_manual': False,
        'exposure_time_abs': 156,
        'dynamic_fps': False,  # 允許相機為了亮度自動降低 FPS
        'awb_auto': True,      # 自動白平衡，讓顏色自動調整
        'af_auto': True,       # 連續自動對焦
        'buffersize': 1,       # 增大會增加延遲
        
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
        # Node(
        #     package='camera',
        #     executable='sobel',
        #     name='sobel',
        #     output='screen'
        # ),
        
        # liquid_level
        Node(
            package='camera',
            executable='liquid_level',
            name='liquid_level',
            output='screen'
        ),
        
        # # liquid_level_adjustment
        # Node(
        #     package='camera',
        #     executable='liquid_level_adj',
        #     name='liquid_level_adj',
        #     output='screen'
        # ),
        
        # # fine line
        # Node(
        #     package='camera',
        #     executable='findLine',
        #     name='findLine',
        #     output='screen'
        # ),
        
        
    ])