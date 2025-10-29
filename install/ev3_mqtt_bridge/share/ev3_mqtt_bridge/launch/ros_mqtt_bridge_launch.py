from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    broker_ip_arg   = DeclareLaunchArgument('broker_ip',   default_value='10.89.89.53')
    username_arg    = DeclareLaunchArgument('username',    default_value='lego')
    password_arg    = DeclareLaunchArgument('password',    default_value='lego')
    broker_port_arg = DeclareLaunchArgument('broker_port', default_value='1883')

    # --- LED ---
    bridge_led = Node(
        package='ev3_mqtt_bridge',
        executable='ros_mqtt_bridge',
        name='bridge_led',
        output='screen',
        parameters=[{
            'broker_ip':   LaunchConfiguration('broker_ip'),
            'broker_port': LaunchConfiguration('broker_port'),
            'username':    LaunchConfiguration('username'),
            'password':    LaunchConfiguration('password'),
            
            # ROS -> MQTT
            'ros_sub_topic':  'ev3/light_cmd',
            'mqtt_pub_topic': 'ev3/light',
            # MQTT -> ROS
            'mqtt_sub_topic': 'ev3/status',
            'ros_pub_topic':  'ev3/status_feedback',
        }]
    )
    
    # --- MotorA ---
    bridge_motorA = Node(
        package='ev3_mqtt_bridge',
        executable='ros_mqtt_bridge',
        name='bridge_motorA',
        output='screen',
        parameters=[{
            'broker_ip':   LaunchConfiguration('broker_ip'),
            'broker_port': LaunchConfiguration('broker_port'),
            'username':    LaunchConfiguration('username'),
            'password':    LaunchConfiguration('password'),

            # ROS -> MQTT（給 EV3 motorA）
            'ros_sub_topic':  'ev3/motorA_cmd',
            'mqtt_pub_topic': 'ev3/motorA/cmd',
            # MQTT -> ROS（EV3 回報）
            'mqtt_sub_topic': 'ev3/motorA/status',
            'ros_pub_topic':  'ev3/motorA_status',
        }]
    )

    # controller_node = Node(
    #     package='ev3_mqtt_bridge',
    #     executable='led_controller',
    #     name='led_controller',
    #     output='screen'
    # )

    # --- 控制節點（發馬達指令 + 觀察回報） ---
    controller = Node(
        package='ev3_mqtt_bridge',
        executable='motorA_controller',
        name='motorA_controller',
        output='screen',
        parameters=[{
            'ros_cmd_topic':   'ev3/motorA_cmd',
            'ros_status_topic':'ev3/motorA_status',
        }]
    )
    
    return LaunchDescription([
        broker_ip_arg, username_arg, password_arg, broker_port_arg,
        bridge_led,
        bridge_motorA,
        controller,
    ])


