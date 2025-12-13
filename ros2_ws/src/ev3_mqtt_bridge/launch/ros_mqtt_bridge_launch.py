from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def bridge_node(ns, name, ros_sub, mqtt_pub, mqtt_sub, ros_pub):
    return Node(
        package='ev3_mqtt_bridge',
        executable='ros_mqtt_bridge',
        name=name,
        namespace=ns,
        output='screen',
        parameters=[{
            'broker_ip':   LaunchConfiguration('broker_ip'),
            'broker_port': LaunchConfiguration('broker_port'),
            'username':    LaunchConfiguration('username'),
            'password':    LaunchConfiguration('password'),
            'ros_sub_topic':  ros_sub,
            'mqtt_pub_topic': mqtt_pub,
            'mqtt_sub_topic': mqtt_sub,
            'ros_pub_topic':  ros_pub,
        }]
    )


def motor_action_node(ns, ros_cmd, ros_status, action_name, node_name):
    return Node(
        package='ev3_mqtt_bridge',
        executable='motor_action',
        name=node_name,
        namespace=ns,
        output='screen',
        parameters=[{
            'ros_cmd_topic': ros_cmd,
            'ros_status_topic': ros_status,
            'action_name': action_name,
        }]
    )


def motor_controller(ns, motor_id, joint, ratio):
    return Node(
        package='ev3_mqtt_bridge',
        executable='motor_controller',
        name=f'motor{joint}_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'motor_id': motor_id,
            'cmd_input_topic': f'motor/motor{joint}_cmd_in',
            'action_name': f'motor{joint}_action',
            'ratio': ratio,
            'speed': 300.0,
        }]
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('broker_ip', default_value='172.20.10.4'),
        DeclareLaunchArgument('broker_port', default_value='1883'),
        DeclareLaunchArgument('username', default_value='lego'),
        DeclareLaunchArgument('password', default_value='lego'),

        # =======================
        # DEBUG: ONLY EV3A / MotorA
        # =======================
        bridge_node(
            'ev3A', 'bridge_motorA',
            'motor/motorA_cmd',
            'ev3A/motorA/cmd',
            'ev3A/motorA/status',
            'motor/motorA_status'
        ),

        motor_action_node(
            'ev3A',
            'motor/motorA_cmd',
            'motor/motorA_status',
            'motorA_action',
            'motorA_action_server'
        ),

        motor_controller(
            'ev3A',
            motor_id=1,   # EV3 port A
            joint='A',
            ratio=-7.0
        ),
    ])
