#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class RosMqttBridge(Node):
    def __init__(self):
        super().__init__('ros_mqtt_bridge')

        # --- 宣告參數（由 launch 傳入） ---
        self.declare_parameter('broker_ip')
        self.declare_parameter('broker_port')
        self.declare_parameter('username')
        self.declare_parameter('password')
        self.declare_parameter('mqtt_pub_topic')   # ROS → MQTT
        self.declare_parameter('mqtt_sub_topic')   # MQTT → ROS
        self.declare_parameter('ros_pub_topic')    # MQTT → ROS
        self.declare_parameter('ros_sub_topic')    # ROS → MQTT

        # --- 取得參數 ---
        broker_ip   = self.get_parameter('broker_ip').get_parameter_value().string_value
        broker_port = self.get_parameter('broker_port').get_parameter_value().integer_value
        username    = self.get_parameter('username').get_parameter_value().string_value
        password    = self.get_parameter('password').get_parameter_value().string_value

        self.mqtt_pub_topic = self.get_parameter('mqtt_pub_topic').get_parameter_value().string_value
        self.mqtt_sub_topic = self.get_parameter('mqtt_sub_topic').get_parameter_value().string_value
        self.ros_pub_topic  = self.get_parameter('ros_pub_topic').get_parameter_value().string_value
        self.ros_sub_topic  = self.get_parameter('ros_sub_topic').get_parameter_value().string_value

        # --- 建立 ROS publisher / subscriber ---
        # MQTT → ROS：收到 MQTT 訊息後轉發到 ROS
        self.ros_publisher = self.create_publisher(String, self.ros_pub_topic, 10)

        # ROS → MQTT：ROS 訂閱後轉發到 MQTT
        self.ros_subscriber = self.create_subscription(String, self.ros_sub_topic, self.on_ros_msg, 10)

        # --- 建立 MQTT 客戶端 ---
        self.client = mqtt.Client()
        if username:
            self.client.username_pw_set(username, password)

        # 設定 MQTT 回呼
        self.client.on_connect = self.on_mqtt_connect
        self.client.on_message = self.on_mqtt_msg

        # 連線並啟動背景執行緒
        self.client.connect(broker_ip, broker_port, 60)
        self.client.loop_start()

        self.get_logger().info(
            f'Bridge running:\n'
            f'  ROS "{self.ros_sub_topic}" → MQTT "{self.mqtt_pub_topic}"\n'
            f'  MQTT "{self.mqtt_sub_topic}" → ROS "{self.ros_pub_topic}"\n'
            f'  Broker: {broker_ip}:{broker_port}'
        )

    # ---------------- ROS → MQTT ----------------
    def on_ros_msg(self, msg: String):
        payload = msg.data.strip()
        self.client.publish(self.mqtt_pub_topic, payload)
        self.get_logger().info(f'[ROS→MQTT] {self.mqtt_pub_topic} = "{payload}"')

    # ---------------- MQTT → ROS ----------------
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f'Connected to MQTT broker, subscribing "{self.mqtt_sub_topic}"')
            client.subscribe(self.mqtt_sub_topic)
        else:
            self.get_logger().error(f'Failed to connect MQTT, code={rc}')

    def on_mqtt_msg(self, client, userdata, msg):
        payload = msg.payload.decode().strip()
        ros_msg = String()
        ros_msg.data = payload
        self.ros_publisher.publish(ros_msg)
        self.get_logger().info(f'[MQTT→ROS] {msg.topic} = "{payload}"')

def main(args=None):
    rclpy.init(args=args)
    node = RosMqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



