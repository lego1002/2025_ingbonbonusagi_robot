#!/usr/bin/env python3
import json
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# ✅ 重點：相對 import（同一個 package 裡）
from .ik_test import (
    ik_constrained,
    check_joint_limits,
    report_limit_violations,
    q_lims,
    DEG,
)

JSON_PATH = "/home/lego/Desktop/2025_ingbonbonusagi_robot/ros2_ws/path.json"

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

def clamp_to_joint_limits(q, q_lims):
    q2 = q.copy()
    for i in range(len(q2)):
        lo, hi = q_lims[i, 0], q_lims[i, 1]
        q2[i] = float(np.clip(q2[i], lo, hi))
    return q2

def interpolate_joint_path(q_start, q_end, steps):
    # 回傳 list(np.array shape(5,))
    out = []
    for i in range(1, steps + 1):
        s = i / steps
        out.append((1.0 - s) * q_start + s * q_end)
    return out

class IKPathPlayer(Node):
    def __init__(self):
        super().__init__("ik_path_player")

        # motor cmd publishers (deg)
        self.motor_pub = {
            "A": self.create_publisher(Float64, "/motor/motorA_cmd_in", 10),
            "B": self.create_publisher(Float64, "/motor/motorB_cmd_in", 10),
            "C": self.create_publisher(Float64, "/motor/motorC_cmd_in", 10),
            "D": self.create_publisher(Float64, "/motor/motorD_cmd_in", 10),
            "E": self.create_publisher(Float64, "/motor/motorE_cmd_in", 10),
            "F": self.create_publisher(Float64, "/motor/motorF_cmd_in", 10),
        }

        # rviz joint states (rad)
        self.joint_state_pub = self.create_publisher(JointState, "/ik_joint_states", 10)

        # 讀 JSON 路徑
        self.path = self.load_path()
        self.idx = 0

        # 前一姿態（rad, 5 joints）
        self.q_prev = np.zeros(5, dtype=float)

        # 插值軌跡 queue
        self.traj_queue = []

        # 播放參數
        self.publish_hz = 20.0          # 20 Hz
        self.steps_per_segment = 50     # 每段 50 steps（約 2.5 秒/段）
        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

        self.get_logger().info(f"IKPathPlayer started. points={len(self.path)}")

    def load_path(self):
        with open(JSON_PATH, "r") as f:
            data = json.load(f)

        path = []
        for p in data:
            # JSON x,y,z 是 cm → 轉成 m
            xyz = np.array([p["x"], p["y"], p["z"]], dtype=float) * 0.01
            path.append((p.get("comment", ""), xyz))
        return path

    def on_timer(self):
        # 1) 若 queue 有插值點 → 直接 publish 下一步
        if self.traj_queue:
            q = self.traj_queue.pop(0)
            self.publish_motors(q)
            self.publish_joint_states(q)
            self.q_prev = q.copy()
            return

        # 2) queue 空了 → 準備下一個 JSON 點
        if self.idx >= len(self.path):
            # 播完就停在最後一點，不再動
            return

        comment, target = self.path[self.idx]
        self.get_logger().info(f"[{self.idx+1}/{len(self.path)}] target: {comment} xyz(m)={target}")

        q_sol, err, ok = ik_constrained(target, self.q_prev)

        if not ok:
            self.get_logger().warn(f"IK failed. err={err:.6f} m. skip this point.")
            self.idx += 1
            return

        # 先檢查是否超限（這裡只是報告），然後 clamp 保護
        violations = check_joint_limits(q_sol, q_lims)
        if violations:
            self.get_logger().warn("Joint limit violated. Will clamp to limits.")
            # 給你在 terminal 看得懂的報告
            report_limit_violations(violations)

        q_sol = clamp_to_joint_limits(q_sol, q_lims)

        # 建立插值軌跡（連續動作的關鍵）
        self.traj_queue = interpolate_joint_path(self.q_prev, q_sol, self.steps_per_segment)

        self.idx += 1

    def publish_motors(self, q_rad):
        # motor topic 要 deg（你之前這樣設計，我照做）
        q_deg = q_rad / DEG

        mapping = {
            "A": float(q_deg[0]),
            "B": float(q_deg[1]),
            "C": float(q_deg[2]),
            "D": float(q_deg[3]),
            "E": float(q_deg[4]),
            "F": 0.0,   # 你目前 IK 只有 5 軸，F 固定 0
        }

        for k, v in mapping.items():
            msg = Float64()
            msg.data = v
            self.motor_pub[k].publish(msg)

    def publish_joint_states(self, q_rad):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [
            float(q_rad[0]),
            float(q_rad[1]),
            float(q_rad[2]),
            float(q_rad[3]),
            float(q_rad[4]),
            0.0,
        ]
        self.joint_state_pub.publish(msg)

def main():
    rclpy.init()
    node = IKPathPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
