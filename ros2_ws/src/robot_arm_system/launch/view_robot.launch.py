import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robot_arm_system'
    urdf_file_name = 'urdf/lego_arm.urdf'

    # 取得 URDF 檔案路徑 (這裡假設您還沒編譯 install，我們先用絕對路徑測試，或請看備註)
    # 正規做法是編譯後用 get_package_share_directory
    # 為了方便您快速測試，請將下面路徑改成您電腦上的真實路徑
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        urdf_file_name)

    # 讀取 URDF 內容
    # 注意：如果您還沒編譯，這裡可能會報錯。
    # 建議先 colcon build --packages-select robot_arm_system

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', rviz_config_file] # 如果有存設定檔
        )
    ])