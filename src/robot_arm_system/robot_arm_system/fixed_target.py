#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class FixedTargetPublisher(Node):
    def __init__(self):
        super().__init__("fixed_target_pub")

        self.pub = self.create_publisher(
            String, "/camera/target_position", 10
        )

        self.target = {"x": 0.166, "y": 0.05, "z": 0.113}

        self.timer = self.create_timer(0.1, self.publish_target)
        self.get_logger().info("Fixed target publisher started")

    def publish_target(self):
        msg = String()
        msg.data = json.dumps(self.target)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixedTargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
