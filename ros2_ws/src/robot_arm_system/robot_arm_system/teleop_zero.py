#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class JointAngleCLI(Node):
    """
    CLI tool:
    - Input: joint index + desired joint angle (deg, arm angle)
    - Output: publish Float64 to motorX_cmd_in
    """

    def __init__(self):
        super().__init__("joint_angle_cli")

        # publishers for all joints
        self.pub = {
            1: self.create_publisher(Float64, "/ev3A/motor/motorA_cmd_in", 10),
            2: self.create_publisher(Float64, "/ev3A/motor/motorB_cmd_in", 10),
            3: self.create_publisher(Float64, "/ev3A/motor/motorC_cmd_in", 10),
            4: self.create_publisher(Float64, "/ev3B/motor/motorD_cmd_in", 10),
            5: self.create_publisher(Float64, "/ev3B/motor/motorE_cmd_in", 10),
            6: self.create_publisher(Float64, "/ev3B/motor/motorF_cmd_in", 10),
        }

        self.get_logger().info("Joint Angle CLI ready.")
        self.get_logger().info("Usage: <joint_id> <angle_deg>")
        self.get_logger().info("Example: 5 90")

        self.run_cli()

    def run_cli(self):
        while rclpy.ok():
            try:
                raw = input("joint angle> ").strip()
                if raw in ("q", "quit", "exit"):
                    break

                joint_id, angle = raw.split()
                joint_id = int(joint_id)
                angle = float(angle)

                if joint_id not in self.pub:
                    print("❌ joint_id must be 1~6")
                    continue

                msg = Float64()
                msg.data = angle
                self.pub[joint_id].publish(msg)

                print(f"✅ Sent: joint {joint_id} → {angle:.2f} deg")

            except Exception as e:
                print(f"❌ error: {e}")

        self.get_logger().info("Exiting Joint Angle CLI")


def main():
    rclpy.init()
    node = JointAngleCLI()
    rclpy.shutdown()
