#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from .ik_test import ik_constrained, check_joint_limits, q_lims, DEG

# ----------------------------
# helpers
# ----------------------------

def smoothstep(s):
    return s * s * (3.0 - 2.0 * s)


def interp_joint_space(q0, q1, step_deg):
    dq = q1 - q0
    steps = max(1, int(np.ceil(np.max(np.abs(dq)) / step_deg)))
    return [q0 + smoothstep(i / steps) * dq for i in range(1, steps + 1)]


def rate_limit(prev, target, vmax, dt):
    dq = target - prev
    return prev + np.clip(dq, -vmax * dt, vmax * dt)


def quantize(q, step):
    return np.round(q / step) * step


# ----------------------------
# main
# ----------------------------

class PathIKExecutor(Node):

    def __init__(self):
        super().__init__("path_ik_executor")

        self.declare_parameter("json_path", "test_point.json")
        self.declare_parameter("joint_step_deg", 3.0)
        self.declare_parameter("v_max_deg_s", 8.0)
        self.declare_parameter("dt", 0.05)

        self.json_path = self.get_parameter("json_path").value
        self.step = float(self.get_parameter("joint_step_deg").value)
        self.vmax = float(self.get_parameter("v_max_deg_s").value)
        self.dt = float(self.get_parameter("dt").value)

        # publishers
        self.pub = {
            "A": self.create_publisher(Float64, "/ev3A/motor/motorA_cmd_in", 10),
            "B": self.create_publisher(Float64, "/ev3A/motor/motorB_cmd_in", 10),
            "C": self.create_publisher(Float64, "/ev3A/motor/motorC_cmd_in", 10),
            "D": self.create_publisher(Float64, "/ev3B/motor/motorD_cmd_in", 10),
            "E": self.create_publisher(Float64, "/ev3B/motor/motorE_cmd_in", 10),
            "F": self.create_publisher(Float64, "/ev3B/motor/motorF_cmd_in", 10),
        }

        self.js_pub = self.create_publisher(JointState, "/ik_joint_states", 10)

        self.last = np.full(6, np.nan)
        self.deadband = 0.3
        self.quant = 0.2

        self.timer = self.create_timer(0.2, self.run_once)
        self.started = False

    def run_once(self):
        if self.started:
            return
        self.started = True

        pts = json.load(open(self.json_path))
        xyzs = [np.array(pts[k], float) for k in sorted(pts, key=int)]
        self.get_logger().info(f"Loaded {len(xyzs)} points")

        q_targets = []
        q_guess = np.zeros(5)

        for xyz in xyzs:
            q5, _, ok = ik_constrained(xyz, q_guess)
            if not ok or check_joint_limits(q5, q_lims):
                self.get_logger().error(f"IK failed at {xyz.tolist()}")
                return

            j1, j2, j3 = q5[:3] / DEG
            j5 = -(j2 + j3)     # ⭐ 末端水平
            q6 = 0.0            # 夾爪暫時固定
            q4 = 0.0            # wrist roll 暫不用

            q_targets.append(np.array([j1, j2, j3, q4, j5, q6]))
            q_guess = q5

        q = q_targets[0]
        self.publish(q)

        for i in range(len(q_targets) - 1):
            seg = interp_joint_space(q_targets[i], q_targets[i+1], self.step)
            for p in seg:
                q = rate_limit(q, p, self.vmax, self.dt)
                self.publish(q)
                time.sleep(self.dt)

        self.get_logger().info("Trajectory finished")

    def publish(self, q):
        q = quantize(q, self.quant)

        if not np.isnan(self.last).any():
            if np.max(np.abs(q - self.last)) < self.deadband:
                return

        for k, v in zip("ABCDEF", q):
            self.pub[k].publish(Float64(data=float(v)))

        self.last = q.copy()

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint{i+1}" for i in range(6)]
        js.position = (q * DEG).tolist()
        self.js_pub.publish(js)


def main():
    rclpy.init()
    rclpy.spin(PathIKExecutor())
    rclpy.shutdown()
