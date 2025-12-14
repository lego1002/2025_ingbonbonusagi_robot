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
            'hold': True,   # ★ 關鍵：啟用 hold，防止垂下
        }]
    )


def motor_controller_node(ns, motor_id, joint, ratio):
    return Node(
        package='ev3_mqtt_bridge',
        executable='motor_controller',
        name=f'motor{joint}_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'motor_id': motor_id,  # EV3 port: 1=A,2=B,3=C
            'cmd_input_topic': f'motor/motor{joint}_cmd_in',
            'action_name': f'motor{joint}_action',
            'ratio': ratio,
            'speed': 300.0,
            'streaming_move_like': True,
            'hold_enabled': True,
            'hold_duty': 8,
            'hold_period_sec': 0.4,

        }]
    )


def generate_launch_description():
    return LaunchDescription([
        # =====================
        # MQTT Broker args
        # =====================
        DeclareLaunchArgument('broker_ip', default_value='172.20.10.4'),
        DeclareLaunchArgument('broker_port', default_value='1883'),
        DeclareLaunchArgument('username', default_value='lego'),
        DeclareLaunchArgument('password', default_value='lego'),

        # =========================================================
        # EV3A → joint1,2,3  (motor A,B,C)
        # =========================================================
        bridge_node(
            'ev3A', 'bridge_motorA',
            'motor/motorA_cmd', 'ev3A/motorA/cmd',
            'ev3A/motorA/status', 'motor/motorA_status'
        ),
        motor_action_node(
            'ev3A', 'motor/motorA_cmd', 'motor/motorA_status',
            'motorA_action', 'motorA_action_server'
        ),
        motor_controller_node('ev3A', 1, 'A', -7.0),

        bridge_node(
            'ev3A', 'bridge_motorB',
            'motor/motorB_cmd', 'ev3A/motorB/cmd',
            'ev3A/motorB/status', 'motor/motorB_status'
        ),
        motor_action_node(
            'ev3A', 'motor/motorB_cmd', 'motor/motorB_status',
            'motorB_action', 'motorB_action_server'
        ),
        motor_controller_node('ev3A', 2, 'B', -70.0 / 9.0),

        bridge_node(
            'ev3A', 'bridge_motorC',
            'motor/motorC_cmd', 'ev3A/motorC/cmd',
            'ev3A/motorC/status', 'motor/motorC_status'
        ),
        motor_action_node(
            'ev3A', 'motor/motorC_cmd', 'motor/motorC_status',
            'motorC_action', 'motorC_action_server'
        ),
        motor_controller_node('ev3A', 3, 'C', 5.0),

        # =========================================================
        # EV3B → joint4,5,6  (motor D,E,F)
        # =========================================================
        bridge_node(
            'ev3B', 'bridge_motorD',
            'motor/motorD_cmd', 'ev3B/motorD/cmd',
            'ev3B/motorD/status', 'motor/motorD_status'
        ),
        motor_action_node(
            'ev3B', 'motor/motorD_cmd', 'motor/motorD_status',
            'motorD_action', 'motorD_action_server'
        ),
        motor_controller_node('ev3B', 1, 'D', 7.0),

        bridge_node(
            'ev3B', 'bridge_motorE',
            'motor/motorE_cmd', 'ev3B/motorE/cmd',
            'ev3B/motorE/status', 'motor/motorE_status'
        ),
        motor_action_node(
            'ev3B', 'motor/motorE_cmd', 'motor/motorE_status',
            'motorE_action', 'motorE_action_server'
        ),
        motor_controller_node('ev3B', 2, 'E', 1.0),

        bridge_node(
            'ev3B', 'bridge_motorF',
            'motor/motorF_cmd', 'ev3B/motorF/cmd',
            'ev3B/motorF/status', 'motor/motorF_status'
        ),
        motor_action_node(
            'ev3B', 'motor/motorF_cmd', 'motor/motorF_status',
            'motorF_action', 'motorF_action_server'
        ),
        motor_controller_node('ev3B', 3, 'F', 24.0),
    ])
