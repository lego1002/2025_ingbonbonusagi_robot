#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Ev3Controller(Node):
    def __init__(self):
        super().__init__('ev3_controller')

        # Publisher: 控制 EV3 LED
        self.pub = self.create_publisher(String, 'ev3/light_cmd', 10)

        # Subscriber: 接收 EV3 狀態
        self.sub = self.create_subscription(String, 'ev3/status_feedback', self.status_callback, 10)

        # 啟動測試計時器
        self.timer = self.create_timer(2.0, self.test_sequence)
        self.step = 0

    def status_callback(self, msg):
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
    node = Ev3Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
