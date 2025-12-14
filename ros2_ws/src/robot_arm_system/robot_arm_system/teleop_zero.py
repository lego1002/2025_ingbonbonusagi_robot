#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32


class JointAngleCLI(Node):
    """
    CLI:
      input: <joint_id> <angle_deg> <speed>
      example: 5 90 20
    """

    def __init__(self):
        super().__init__("joint_angle_cli")

        # angle publishers
        self.angle_pub = {
            1: self.create_publisher(Float64, "/ev3A/motor/motorA_cmd_in", 10),
            2: self.create_publisher(Float64, "/ev3A/motor/motorB_cmd_in", 10),
            3: self.create_publisher(Float64, "/ev3A/motor/motorC_cmd_in", 10),
            4: self.create_publisher(Float64, "/ev3B/motor/motorD_cmd_in", 10),
            5: self.create_publisher(Float64, "/ev3B/motor/motorE_cmd_in", 10),
            6: self.create_publisher(Float64, "/ev3B/motor/motorF_cmd_in", 10),
        }

        # speed publishers（motor_controller 要接）
        self.speed_pub = {
            1: self.create_publisher(Int32, "/ev3A/motor/motorA_speed", 10),
            2: self.create_publisher(Int32, "/ev3A/motor/motorB_speed", 10),
            3: self.create_publisher(Int32, "/ev3A/motor/motorC_speed", 10),
            4: self.create_publisher(Int32, "/ev3B/motor/motorD_speed", 10),
            5: self.create_publisher(Int32, "/ev3B/motor/motorE_speed", 10),
            6: self.create_publisher(Int32, "/ev3B/motor/motorF_speed", 10),
        }

        self.get_logger().info("Joint Angle CLI (with speed)")
        self.get_logger().info("Usage: <joint_id> <angle_deg> <speed>")
        self.get_logger().info("Example: 5 0 15   (slow homing)")
        self.get_logger().info("Type 'q' to quit")

        self.run_cli()

    def run_cli(self):
        while rclpy.ok():
            try:
                raw = input("joint angle speed> ").strip()
                if raw in ("q", "quit", "exit"):
                    break

                joint_id, angle, speed = raw.split()
                joint_id = int(joint_id)
                angle = float(angle)
                speed = int(speed)

                if joint_id not in self.angle_pub:
                    print("❌ joint_id must be 1~6")
                    continue

                self.angle_pub[joint_id].publish(Float64(data=angle))
                self.speed_pub[joint_id].publish(Int32(data=speed))

                print(
                    f"✅ joint {joint_id} → {angle:.2f} deg @ speed {speed}"
                )

            except Exception as e:
                print(f"❌ error: {e}")

        self.get_logger().info("CLI exit")


def main():
    rclpy.init()
    node = JointAngleCLI()
    rclpy.shutdown()
