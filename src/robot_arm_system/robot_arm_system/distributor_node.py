#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DistributorNode(Node):
    def __init__(self):
        super().__init__("ik_distributor")

        self.sub = self.create_subscription(
            String, "/ik/joint_targets", self.on_ik, 10
        )

        self.pubs = [
            self.create_publisher(String, "/ev3A/motorA_cmd_in", 10),
            self.create_publisher(String, "/ev3A/motorB_cmd_in", 10),
            self.create_publisher(String, "/ev3A/motorC_cmd_in", 10),
            self.create_publisher(String, "/ev3B/motorD_cmd_in", 10),
            self.create_publisher(String, "/ev3B/motorE_cmd_in", 10),
        ]

        self.speed = 300
        self.get_logger().info("Distributor Node started.")

    def on_ik(self, msg):
        data = json.loads(msg.data)
        q = [data["q1"], data["q2"], data["q3"], data["q4"], data["q5"]]

        for i in range(5):
            out = String()
            out.data = f"abs {q[i]} {self.speed}"
            self.pubs[i].publish(out)

        self.get_logger().info(f"Sent commands → {q}")


def main(args=None):
    rclpy.init(args=args)
    node = DistributorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
