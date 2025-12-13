from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


# 主機EV3 mqtt 橋接
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
            # ROS ↔ MQTT 對應（注意：ROS 是相對名，會自動帶上 namespace）
            'ros_sub_topic':  ros_sub,   # ROS → MQTT
            'mqtt_pub_topic': mqtt_pub,  # 發到哪個 MQTT topic
            'mqtt_sub_topic': mqtt_sub,  # MQTT → ROS
            'ros_pub_topic':  ros_pub,
        }]
    )


# led 控制測試，確認訊息是否互通
def led_controller_node(ns):
    return Node(
        package='ev3_mqtt_bridge',
        executable='led_controller',
        name='led_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'ros_cmd_topic':   'led/light_cmd',
            'ros_status_topic': 'led/status_feedback',
        }]
    )


# 馬達動作模組（Action Server）
def motor_action_node(ns, ros_cmd, ros_status, action_name, node_name):
    return Node(
        package='ev3_mqtt_bridge',
        executable='motor_action',
        name=node_name,          # 每顆馬達的 node name 不同
        namespace=ns,
        output='screen',
        parameters=[{
            'ros_cmd_topic':   ros_cmd,
            'ros_status_topic': ros_status,
            'action_name':      action_name,
        }]
    )


# 馬達動作控制（Controller）
def motor_controller_node(ns, exe_name, motor_id, input_topic, action_name, ratio):
    return Node(
        package='ev3_mqtt_bridge',
        executable=exe_name,
        name=exe_name,
        namespace=ns,
        output='screen',
        parameters=[{
            'demo_mode': True,               # 測試先 True
            'cmd_input_topic': input_topic,  # 之後 demo_mode=False 會用到
            'motor_id': motor_id,
            'action_name': action_name,      # 告訴 controller 要連哪個 action
            'ratio': ratio,                  # ★ 齒輪比（軸角度 → 馬達角度）
        }]
    )


def generate_launch_description():
    broker_ip_arg   = DeclareLaunchArgument('broker_ip',   default_value='10.13.209.64')
    username_arg    = DeclareLaunchArgument('username',    default_value='lego')
    password_arg    = DeclareLaunchArgument('password',    default_value='lego')
    broker_port_arg = DeclareLaunchArgument('broker_port', default_value='1883')

    # === EV3A: LED === (for test)
    # ROS topics:   /ev3A/led/light_cmd  <->  /ev3A/led/status_feedback
    # MQTT topics:  ev3A/led/light       <->  ev3A/led/status
    bridge_led_A = bridge_node(
        ns='ev3A', name='bridge_led',
        ros_sub='led/light_cmd',
        mqtt_pub='ev3A/led/light',
        mqtt_sub='ev3A/led/status',
        ros_pub='led/status_feedback'
    )
    led_ctrl_A = led_controller_node('ev3A')

    # === EV3B: LED === (for test)
    bridge_led_B = bridge_node(
        ns='ev3B', name='bridge_led',
        ros_sub='led/light_cmd',
        mqtt_pub='ev3B/led/light',
        mqtt_sub='ev3B/led/status',
        ros_pub='led/status_feedback'
    )
    led_ctrl_B = led_controller_node('ev3B')

    # ===============
    # EV3A: MotorA 
    # ===============
    bridge_motorA = bridge_node(
        ns='ev3A', name='bridge_motorA',
        ros_sub='motor/motorA_cmd',        # ROS→MQTT 指令
        mqtt_pub='ev3A/motorA/cmd',        # EV3A 端要訂閱這個
        mqtt_sub='ev3A/motorA/status',     # EV3A 回報
        ros_pub='motor/motorA_status'      # MQTT→ROS 指令
    )
    actionA_name = 'motorA_action'
    motor_action_A = motor_action_node(
        'ev3A',
        'motor/motorA_cmd',
        'motor/motorA_status',
        actionA_name,
        'motorA_action_server'
    )
    motorA_ctrl = motor_controller_node(
        'ev3A',
        'motorA_controller',
        1,                              # motor id
        'motor/motorA_cmd_in',          # 輸入角度
        actionA_name,
        -7.0                               # gear ratio
    )

    # ===============
    # EV3A: MotorB 
    # ===============
    bridge_motorB = bridge_node(
        ns='ev3A', name='bridge_motorB',
        ros_sub='motor/motorB_cmd',
        mqtt_pub='ev3A/motorB/cmd',
        mqtt_sub='ev3A/motorB/status',
        ros_pub='motor/motorB_status'
    )
    actionB_name = 'motorB_action'
    motor_action_B = motor_action_node(
        'ev3A',
        'motor/motorB_cmd',
        'motor/motorB_status',
        actionB_name,
        'motorB_action_server'
    )
    motorB_ctrl = motor_controller_node(
        'ev3A',
        'motorB_controller',
        2,
        'motor/motorB_cmd_in',
        actionB_name,
        -70/9
    )

    # ===============
    # EV3A: MotorC 
    # ===============
    bridge_motorC = bridge_node(
        ns='ev3A', name='bridge_motorC',
        ros_sub='motor/motorC_cmd',
        mqtt_pub='ev3A/motorC/cmd',
        mqtt_sub='ev3A/motorC/status',
        ros_pub='motor/motorC_status'
    )
    actionC_name = 'motorC_action'
    motor_action_C = motor_action_node(
        'ev3A',
        'motor/motorC_cmd',
        'motor/motorC_status',
        actionC_name,
        'motorC_action_server'
    )
    motorC_ctrl = motor_controller_node(
        'ev3A',
        'motorC_controller',
        3,
        'motor/motorC_cmd_in',
        actionC_name,
        5.0
    )
    
    # ===============
    # EV3B: MotorD 
    # ===============
    bridge_motorD = bridge_node(
        ns='ev3B', name='bridge_motorD',
        ros_sub='motor/motorD_cmd',
        mqtt_pub='ev3B/motorD/cmd',
        mqtt_sub='ev3B/motorD/status',
        ros_pub='motor/motorD_status'
    )
    actionD_name = 'motorD_action'
    motor_action_D = motor_action_node(
        'ev3B',
        'motor/motorD_cmd',
        'motor/motorD_status',
        actionD_name,
        'motorD_action_server'
    )
    motorD_ctrl = motor_controller_node(
        'ev3B',
        'motorD_controller',
        4,
        'motor/motorD_cmd_in',
        actionD_name,
        7.0
    )
    
    # ===============
    # EV3B: MotorE 
    # ===============
    bridge_motorE = bridge_node(
        ns='ev3B', name='bridge_motorE',
        ros_sub='motor/motorE_cmd',
        mqtt_pub='ev3B/motorE/cmd',
        mqtt_sub='ev3B/motorE/status',
        ros_pub='motor/motorE_status'
    )
    actionE_name = 'motorE_action'
    motor_action_E = motor_action_node(
        'ev3B',
        'motor/motorE_cmd',
        'motor/motorE_status',
        actionE_name,
        'motorE_action_server'
    )
    motorE_ctrl = motor_controller_node(
        'ev3B',
        'motorE_controller',
        5,
        'motor/motorE_cmd_in',
        actionE_name,
        1.0
    )
    
    # ===============
    # EV3B: MotorF 
    # ===============
    bridge_motorF = bridge_node(
        ns='ev3B', name='bridge_motorF',
        ros_sub='motor/motorF_cmd',
        mqtt_pub='ev3B/motorF/cmd',
        mqtt_sub='ev3B/motorF/status',
        ros_pub='motor/motorF_status'
    )
    actionF_name = 'motorF_action'
    motor_action_F = motor_action_node(
        'ev3B',
        'motor/motorF_cmd',
        'motor/motorF_status',
        actionF_name,
        'motorF_action_server'
    )
    motorF_ctrl = motor_controller_node(
        'ev3B',
        'motorF_controller',
        6, 
        'motor/motorF_cmd_in',
        actionF_name,
        24.0
    )

    return LaunchDescription([
        broker_ip_arg, username_arg, password_arg, broker_port_arg,
        # LED (test communication)
        # bridge_led_A, led_ctrl_A,
        # bridge_led_B, led_ctrl_B,

        # # MotorA (ev3A)
        bridge_motorA, motor_action_A, motorA_ctrl,
        # # MotorB (ev3A)
        # bridge_motorB, motor_action_B, motorB_ctrl,
        # # MotorC (ev3A)
        # bridge_motorC, motor_action_C, motorC_ctrl,
        # MotorD (ev3B)
        bridge_motorD, motor_action_D, motorD_ctrl,
        # MotorE (ev3B)
        # bridge_motorE, motor_action_E, motorE_ctrl,
        # MotorF (ev3B)
        # bridge_motorF, motor_action_F, motorF_ctrl,
    ])
    