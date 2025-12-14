#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from .ik_test import (
    ik_constrained,
    check_joint_limits,
    q_lims,
    DEG,
)

# =========================
# Helper functions
# =========================

def smoothstep(s: float) -> float:
    """C1 smooth interpolation"""
    return s * s * (3.0 - 2.0 * s)


def interp_joint_space(q0: np.ndarray, q1: np.ndarray, step_deg: float):
    """
    Joint-space interpolation using smoothstep
    q0, q1 : deg
    """
    dq = q1 - q0
    max_dq = np.max(np.abs(dq))
    steps = max(1, int(np.ceil(max_dq / step_deg)))

    traj = []
    for i in range(1, steps + 1):
        s = smoothstep(i / steps)
        traj.append(q0 + s * dq)
    return traj


def rate_limit(prev: np.ndarray, target: np.ndarray, v_max: float, dt: float):
    """
    Velocity limiting (deg/s)
    """
    dq = target - prev
    max_step = v_max * dt
    dq = np.clip(dq, -max_step, max_step)
    return prev + dq


# =========================
# Main Node
# =========================

class PathIKExecutor(Node):

    def __init__(self):
        super().__init__("path_ik_executor")

        # -------- Parameters --------
        self.declare_parameter("json_path", "test_point.json")
        self.declare_parameter("joint_step_deg", 2.0)
        self.declare_parameter("v_max_deg_s", 5.0)
        self.declare_parameter("dt", 0.08)

        self.json_path = self.get_parameter("json_path").value
        self.joint_step_deg = float(self.get_parameter("joint_step_deg").value)
        self.v_max = float(self.get_parameter("v_max_deg_s").value)
        self.dt = float(self.get_parameter("dt").value)

        # -------- Publishers (EV3A only) --------
        self.pub_A = self.create_publisher(Float64, "/ev3A/motor/motorA_cmd_in", 10)
        self.pub_B = self.create_publisher(Float64, "/ev3A/motor/motorB_cmd_in", 10)
        self.pub_C = self.create_publisher(Float64, "/ev3A/motor/motorC_cmd_in", 10)

        self.js_pub = self.create_publisher(JointState, "/ik_joint_states", 10)

        # -------- Run once --------
        self.started = False
        self.timer = self.create_timer(0.2, self.run_once)

    # =========================
    # Core logic
    # =========================

    def run_once(self):
        if self.started:
            return
        self.started = True

        # ---- Load JSON ----
        with open(self.json_path, "r") as f:
            raw = json.load(f)

        points = [np.array(raw[k], float) for k in sorted(raw.keys(), key=int)]
        self.get_logger().info(f"Loaded {len(points)} points")

        # ---- IK for each point ----
        q_targets = []
        q_guess = np.zeros(5)

        for idx, xyz in enumerate(points):
            q_rad, err, ok = ik_constrained(xyz, q_guess)
            if not ok or check_joint_limits(q_rad, q_lims):
                self.get_logger().error(f"IK failed at point {idx}: {xyz.tolist()}")
                return

            q_deg = q_rad[:3] / DEG  # joint 1~3 only
            q_targets.append(q_deg)
            q_guess = q_rad

            self.get_logger().info(
                f"IK ok {idx}: xyz={xyz.tolist()} q_deg={q_deg.round(2).tolist()}"
            )

        # ---- Execute trajectory ----
        q_cmd = q_targets[0].copy()
        self.publish(q_cmd)
        time.sleep(self.dt)

        for i in range(len(q_targets) - 1):
            seg = interp_joint_space(
                q_targets[i],
                q_targets[i + 1],
                self.joint_step_deg
            )

            for p in seg:
                q_cmd = rate_limit(q_cmd, p, self.v_max, self.dt)
                self.publish(q_cmd)
                time.sleep(self.dt)

            # 🔑 關鍵：段落結束後強制對齊
            q_cmd = q_targets[i + 1].copy()
            self.publish(q_cmd)
            time.sleep(self.dt)

        self.get_logger().info("Trajectory finished")

    # =========================
    # Publish
    # =========================

    def publish(self, q_deg: np.ndarray):
        # EV3A joints
        self.pub_A.publish(Float64(data=float(q_deg[0])))
        self.pub_B.publish(Float64(data=float(q_deg[1])))
        self.pub_C.publish(Float64(data=float(q_deg[2])))

        # For visualization / debug
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["joint1", "joint2", "joint3"]
        js.position = (q_deg * DEG).tolist()
        self.js_pub.publish(js)


# =========================
# Main
# =========================

def main(args=None):
    rclpy.init(args=args)
    node = PathIKExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
