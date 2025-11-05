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
            'ros_status_topic':'led/status_feedback',
        }]
    )
# 馬達動作模組
def motor_service_node(ns):
    return Node(
        package='ev3_mqtt_bridge',
        executable='motor_service',
        name='motor_service',
        namespace=ns,
        output='screen',
        parameters=[{
            'ros_cmd_topic':   'motor/motorA_cmd',
            'ros_status_topic':'motor/motorA_status',
        }]
    )
# ------------
# 馬達動作控制
# ------------
def motorA_controller_node(ns):
    return Node(
        package='ev3_mqtt_bridge',
        executable='motorA_controller',
        name='motorA_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'service_ns': ns,
            'demo_mode':  True,
            'cmd_input_topic': 'motor/motorA_cmd_in',
            'motor_id': 1,
        }]
    )

def motorB_controller_node(ns):
    return Node(
        package='ev3_mqtt_bridge',
        executable='motorB_controller',
        name='motorB_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'service_ns': ns,
            'demo_mode':  True,
            'cmd_input_topic': 'motor/motorB_cmd_in',
            'motor_id': 2,
        }]
    )

def motorC_controller_node(ns):
    return Node(
        package='ev3_mqtt_bridge',
        executable='motorC_controller',
        name='motorC_controller',
        namespace=ns,
        output='screen',
        parameters=[{
            'service_ns': ns,
            'demo_mode':  True,
            'cmd_input_topic': 'motor/motorC_cmd_in',
            'motor_id': 3,
        }]
    )

def generate_launch_description():
    broker_ip_arg   = DeclareLaunchArgument('broker_ip',   default_value='10.89.89.64')
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
    # 1) Bridge (ROS <-> MQTT)
    bridge_motorA = bridge_node(
        ns='ev3A', name='bridge_motorA',
        ros_sub='motor/motorA_cmd',        # ROS→MQTT 指令
        mqtt_pub='ev3A/motorA/cmd',        # EV3A 端要訂閱這個
        mqtt_sub='ev3A/motorA/status',     # EV3A 回報
        ros_pub='motor/motorA_status'
    )
    # 2) Service provider
    motor_service_A = motor_service_node('ev3A')
    # 3) Controller（示範/或轉文字命令→Service 呼叫）
    motorA_ctrl = motorA_controller_node('ev3A')
    
    # ===============
    # EV3A: MotorB 
    # ===============
    bridge_motorB = bridge_node(
        ns='ev3A', name='bridge_motorB',
        ros_sub='motor/motorB_cmd',        # ROS→MQTT 指令
        mqtt_pub='ev3A/motorB/cmd',        # EV3A 端要訂閱這個
        mqtt_sub='ev3A/motorB/status',     # EV3A 回報
        ros_pub='motor/motorB_status'
    )
    motor_service_B = motor_service_node('ev3A')
    motorB_ctrl = motorB_controller_node('ev3A')
    
    # ===============
    # EV3A: MotorC 
    # ===============
    bridge_motorC = bridge_node(
        ns='ev3A', name='bridge_motorC',
        ros_sub='motor/motorC_cmd',        # ROS→MQTT 指令
        mqtt_pub='ev3A/motorC/cmd',        # EV3A 端要訂閱這個
        mqtt_sub='ev3A/motorC/status',     # EV3A 回報
        ros_pub='motor/motorC_status'
    )
    motor_service_C = motor_service_node('ev3A')
    motorC_ctrl = motorC_controller_node('ev3A')
    
    return LaunchDescription([
            broker_ip_arg, username_arg, password_arg, broker_port_arg,
            # LED
            # bridge_led_A, led_ctrl_A,
            # bridge_led_B, led_ctrl_B,
            
            # MotorA (ev3A)
            bridge_motorA, motor_service_A, motorA_ctrl,
            # MotorB (ev3A)
            bridge_motorB, motor_service_B, motorB_ctrl,
            # MotorC (ev3A)
            bridge_motorC, motor_service_C, motorC_ctrl,
            
            
        ])
