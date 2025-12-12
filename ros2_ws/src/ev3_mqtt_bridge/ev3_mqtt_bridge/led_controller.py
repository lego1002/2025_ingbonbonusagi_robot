#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Ev3LedController(Node):
    def __init__(self):
        super().__init__('led_controller')

        # 讓 topic 名由參數決定（交給 launch 管）
        self.declare_parameter('ros_cmd_topic')
        self.declare_parameter('ros_status_topic')

        cmd_topic = self.get_parameter('ros_cmd_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('ros_status_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(String, cmd_topic, 10)
        self.sub = self.create_subscription(String, status_topic, self.status_callback, 10)

        self.get_logger().info(f'[LED Controller] pub="{cmd_topic}", sub="{status_topic}"')

        self.step = 0
        self.timer = self.create_timer(2.0, self.test_sequence)

    def status_callback(self, msg: String):
        self.get_logger().info(f'[EV3 回報] {msg.data}')

    def test_sequence(self):
        colors = ['red', 'green', 'amber', 'off']
        color = colors[self.step % len(colors)]
        msg = String()
        msg.data = color
        self.pub.publish(msg)
        self.get_logger().info(f'[測試] 發送顏色: {color}')
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = Ev3LedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
