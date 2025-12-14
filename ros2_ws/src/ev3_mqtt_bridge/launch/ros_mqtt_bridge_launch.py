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
            'speed': 100.0,
        }]
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('broker_ip', default_value='172.20.10.4'),
        DeclareLaunchArgument('broker_port', default_value='1883'),
        DeclareLaunchArgument('username', default_value='lego'),
        DeclareLaunchArgument('password', default_value='lego'),

        # =======================
        # EV3A / MotorA
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
        
        # =======================
        # EV3A / MotorB
        # =======================
        bridge_node(
            'ev3A', 'bridge_motorB',
            'motor/motorB_cmd',
            'ev3A/motorB/cmd',
            'ev3A/motorB/status',
            'motor/motorB_status'
        ),

        motor_action_node(
            'ev3A',
            'motor/motorB_cmd',
            'motor/motorB_status',
            'motorB_action',
            'motorB_action_server'
        ),

        motor_controller(
            'ev3A',
            motor_id=2,   # EV3 port A
            joint='B',
            ratio=-70/9
        ),
        
        # =======================
        # EV3A / MotorC
        # =======================
        bridge_node(
            'ev3A', 'bridge_motorC',
            'motor/motorC_cmd',
            'ev3A/motorC/cmd',
            'ev3A/motorC/status',
            'motor/motorC_status'
        ),

        motor_action_node(
            'ev3A',
            'motor/motorC_cmd',
            'motor/motorC_status',
            'motorC_action',
            'motorC_action_server'
        ),

        motor_controller(
            'ev3A',
            motor_id=3,   # EV3 port A
            joint='C',
            ratio=5.0
        ),
        
        # =======================
        # EV3B / MotorD
        # =======================
        bridge_node(
            'ev3B', 'bridge_motorD',
            'motor/motorD_cmd',
            'ev3B/motorD/cmd',
            'ev3B/motorD/status',
            'motor/motorD_status'
        ),

        motor_action_node(
            'ev3B',
            'motor/motorD_cmd',
            'motor/motorD_status',
            'motorD_action',
            'motorD_action_server'
        ),

        motor_controller(
            'ev3B',
            motor_id=4,   # EV3 port A
            joint='D',
            ratio=7.0
        ),
        
        # =======================
        # EV3B / MotorE
        # =======================
        bridge_node(
            'ev3B', 'bridge_motorE',
            'motor/motorE_cmd',
            'ev3B/motorE/cmd',
            'ev3B/motorE/status',
            'motor/motorE_status'
        ),

        motor_action_node(
            'ev3B',
            'motor/motorE_cmd',
            'motor/motorE_status',
            'motorE_action',
            'motorE_action_server'
        ),

        motor_controller(
            'ev3B',
            motor_id=5,   # EV3 port A
            joint='E',
            ratio=-5.0
        ),
        
        # =======================
        # EV3B / MotorF
        # =======================
        bridge_node(
            'ev3B', 'bridge_motorF',
            'motor/motorF_cmd',
            'ev3B/motorF/cmd',
            'ev3B/motorF/status',
            'motor/motorF_status'
        ),

        motor_action_node(
            'ev3B',
            'motor/motorF_cmd',
            'motor/motorF_status',
            'motorF_action',
            'motorF_action_server'
        ),

        motor_controller(
            'ev3B',
            motor_id=6,   # EV3 port A
            joint='F',
            ratio=-24.0
        ),
    ])
