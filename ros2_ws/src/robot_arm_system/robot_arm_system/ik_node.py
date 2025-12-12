#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math

class IKNode(Node):
    def __init__(self):
        super().__init__("ik_node")

        self.sub = self.create_subscription(
            String, "/camera/target_position", self.on_target, 10
        )

        self.pub = self.create_publisher(
            String, "/ik/joint_targets", 10
        )

        self.get_logger().info("IK Node started.")

        # DH geometry (meters)
        self.a1 = 0.023
        self.d1 = 0.105
        self.a2 = 0.135
        self.a3 = 0.035

    def on_target(self, msg):
        data = json.loads(msg.data)
        x = float(data["x"])
        y = float(data["y"])
        z = float(data["z"])

        q1, q2, q3, q4, q5 = self.inverse_kinematics(x, y, z)

        out = {
            "q1": math.degrees(q1),
            "q2": math.degrees(q2),
            "q3": math.degrees(q3),
            "q4": math.degrees(q4),
            "q5": math.degrees(q5),
        }

        self.pub.publish(String(data=json.dumps(out)))
        self.get_logger().info(f"IK → {out}")

    def inverse_kinematics(self, x, y, z):
        a1 = self.a1
        d1 = self.d1
        L2 = self.a2
        L3 = self.a3

        # 1) Base rotation
        q1 = math.atan2(y, x)

        # 2) XY plane projection
        r_xy = math.sqrt(x*x + y*y)
        xp = r_xy - a1
        zp = z - d1

        # 3) 2-link IK (shoulder + elbow)
        r = math.sqrt(xp*xp + zp*zp)
        cos_q3 = (r*r - L2*L2 - L3*L3) / (2.0 * L2 * L3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        q3 = math.atan2(+math.sqrt(1 - cos_q3*cos_q3), cos_q3)

        psi = math.atan2(zp, xp)
        phi = math.atan2(L3*math.sin(q3), L2 + L3*math.cos(q3))
        q2 = psi - phi

        # 4) wrist orientation limitation
        q4 = 0.0
        q5 = -(q2 + q3)

        return q1, q2, q3, q4, q5


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
