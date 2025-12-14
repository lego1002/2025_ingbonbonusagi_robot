#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from .ik_test import (
    ik_constrained,
    check_joint_limits,
    q_lims,
    DEG,
)

# ==========
# 工具
# ==========

def _quintic_s(r: float) -> float:
    """
    Quintic time scaling:
      s(r)=10r^3 - 15r^4 + 6r^5
    r in [0,1] -> s in [0,1], and s'(0)=s'(1)=0, s''(0)=s''(1)=0
    """
    r = max(0.0, min(1.0, r))
    return 10.0*r**3 - 15.0*r**4 + 6.0*r**5


def _load_points_from_json(path: str) -> List[np.ndarray]:
    """
    JSON format:
      {
        "0": [x,y,z],
        "1": [x,y,z],
        ...
      }
    unit: meters
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if isinstance(data, list):
        # 兼容 list of dict / list of xyz
        pts = []
        for item in data:
            if isinstance(item, dict) and all(k in item for k in ("x", "y", "z")):
                pts.append(np.array([float(item["x"]), float(item["y"]), float(item["z"])], dtype=float))
            elif isinstance(item, (list, tuple)) and len(item) == 3:
                pts.append(np.array([float(item[0]), float(item[1]), float(item[2])], dtype=float))
            else:
                raise ValueError("Unsupported JSON list item format.")
        return pts

    if not isinstance(data, dict):
        raise ValueError("JSON must be dict or list.")

    # dict: sort by numeric key if possible
    def key_fn(k: str):
        try:
            return int(k)
        except Exception:
            return k

    keys = sorted(list(data.keys()), key=key_fn)
    pts = []
    for k in keys:
        v = data[k]
        if not (isinstance(v, (list, tuple)) and len(v) == 3):
            raise ValueError(f'JSON key "{k}" must be [x,y,z].')
        pts.append(np.array([float(v[0]), float(v[1]), float(v[2])], dtype=float))
    return pts


# ==========
# 主節點
# ==========

class PathIKExecutor(Node):
    """
    Pure Joint-Space execution:
      - No cartesian interpolation
      - No approach/retreat
      - Per keypoint: IK once -> q (5 DOF)
      - Between q_i and q_{i+1}: joint-space smooth interpolation (quintic)
      - Publish joint angles (deg) to motor/*_cmd_in topics
    """

    def __init__(self):
        super().__init__("path_ik_executor")

        # --- Parameters ---
        self.declare_parameter("json_path", "/home/lego/Desktop/2025_ingbonbonusagi_robot/ros2_ws/test_point.json")
        self.declare_parameter("dt", 0.04)               # seconds
        self.declare_parameter("v_max_deg_s", 20.0)      # deg/s (per joint)
        self.declare_parameter("joint_step_deg", 5.0)    # max delta per step (deg)
        self.declare_parameter("settle_sec", 0.0)        # optional wait after each segment
        self.declare_parameter("q_init_deg", [0.0, 0.0, 0.0, 0.0, 0.0])  # IK initial guess (5 DOF)
        self.declare_parameter("j6_deg", 0.0)            # joint6 fixed (if you have motorF etc.)
        self.declare_parameter("loop", False)            # loop path forever

        self.json_path = self.get_parameter("json_path").get_parameter_value().string_value
        self.dt = float(self.get_parameter("dt").value)
        self.v_max_deg_s = float(self.get_parameter("v_max_deg_s").value)
        self.joint_step_deg = float(self.get_parameter("joint_step_deg").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.q_init_deg = list(self.get_parameter("q_init_deg").value)
        self.j6_deg = float(self.get_parameter("j6_deg").value)
        self.loop = bool(self.get_parameter("loop").value)

        # --- Publishers (publish joint angle in DEG) ---
        # These topics match what you showed in `ros2 topic list`
        self.pub = {
            "A": self.create_publisher(Float64, "/ev3A/motor/motorA_cmd_in", 10),
            "B": self.create_publisher(Float64, "/ev3A/motor/motorB_cmd_in", 10),
            "C": self.create_publisher(Float64, "/ev3A/motor/motorC_cmd_in", 10),
            "D": self.create_publisher(Float64, "/ev3B/motor/motorD_cmd_in", 10),
            "E": self.create_publisher(Float64, "/ev3B/motor/motorE_cmd_in", 10),
            "F": self.create_publisher(Float64, "/ev3B/motor/motorF_cmd_in", 10),
        }

        # Load points
        self.key_xyz = _load_points_from_json(self.json_path)
        self.get_logger().info(f"Loaded {len(self.key_xyz)} keypoints from JSON.")

        # Execute
        self._run()

    def _publish_joints_deg(self, q_deg_6: List[float]):
        """
        q_deg_6: [j1..j6] in degrees
        """
        msg = Float64()
        msg.data = float(q_deg_6[0]); self.pub["A"].publish(msg)
        msg = Float64()
        msg.data = float(q_deg_6[1]); self.pub["B"].publish(msg)
        msg = Float64()
        msg.data = float(q_deg_6[2]); self.pub["C"].publish(msg)
        msg = Float64()
        msg.data = float(q_deg_6[3]); self.pub["D"].publish(msg)
        msg = Float64()
        msg.data = float(q_deg_6[4]); self.pub["E"].publish(msg)
        msg = Float64()
        msg.data = float(q_deg_6[5]); self.pub["F"].publish(msg)

    def _ik_for_xyz(self, xyz: np.ndarray, q_guess_rad_5: np.ndarray):
        """
        Returns:
        q_sol (np.ndarray, rad, shape (5,))
        ok (bool)
        """
        q_sol, err, ok = ik_constrained(
            xyz,
            q_init=np.array([
                q_guess_rad_5[0],
                q_guess_rad_5[1],
                q_guess_rad_5[2],
                0.0,
                q_guess_rad_5[4],
            ])
        )

        if not ok:
            self.get_logger().error(
                f"IK failed at xyz={xyz.tolist()} (pos err={err:.6g} m)"
            )
            return q_sol, False

        viol = check_joint_limits(q_sol, q_lims)
        if viol:
            self.get_logger().error(
                f"Joint limit violation at xyz={xyz.tolist()}, joints={viol}"
            )
            return q_sol, False

        # ✅ 關鍵：成功路徑一定要回傳
        return q_sol, True


    def _segment_steps(self, q0_deg: np.ndarray, q1_deg: np.ndarray) -> int:
        dq = np.abs(q1_deg - q0_deg)
        # step constraint
        n_step = int(np.ceil(np.max(dq) / max(1e-6, self.joint_step_deg)))
        # speed constraint: dq <= v_max * dt * steps
        n_vel = int(np.ceil(np.max(dq) / max(1e-6, self.v_max_deg_s * self.dt)))
        n = max(1, n_step, n_vel)
        return n

    def _run_once(self) -> bool:
        # IK for each keypoint
        q_guess_rad = np.array(self.q_init_deg, dtype=float) * DEG  # 5 DOF guess
        q_list_rad: List[np.ndarray] = []

        for i, xyz in enumerate(self.key_xyz):
            q_sol, ok = self._ik_for_xyz(xyz, q_guess_rad)
            if not ok:
                self.get_logger().error("IK path empty / failed.")
                return False
            q_list_rad.append(q_sol.copy())
            q_guess_rad = q_sol  # warm start
            self.get_logger().info(f"IK ok for keypoint {i+1}/{len(self.key_xyz)}: xyz={xyz.tolist()}")

        self.get_logger().info(f"IK succeeded for {len(q_list_rad)} keypoints. Start joint-space execute.")

        # Execute segments
        for seg in range(len(q_list_rad) - 1):
            q0_deg = (q_list_rad[seg] / DEG).astype(float)      # 5
            q1_deg = (q_list_rad[seg + 1] / DEG).astype(float)  # 5

            steps = self._segment_steps(q0_deg, q1_deg)
            self.get_logger().info(
                f"Segment {seg+1}/{len(q_list_rad)-1}: steps={steps}, "
                f"q0={np.round(q0_deg, 2).tolist()}, q1={np.round(q1_deg, 2).tolist()}"
            )

            for k in range(steps + 1):
                r = k / float(steps)
                s = _quintic_s(r)
                qk_deg_5 = q0_deg + s * (q1_deg - q0_deg)

                # Safety check (5 joints)
                viol = check_joint_limits(qk_rad_5, q_lims)
                if viol:
                    self.get_logger().error(
                        f"Joint limit violation during interpolation at seg={seg}, step={k}/{steps}, joints={viol}"
                    )
                    return False

                # Map to 6 joints (j6 fixed)
                qk_deg_6 = [
                    float(qk_deg_5[0]),  # J1
                    float(qk_deg_5[1]),  # J2
                    float(qk_deg_5[2]),  # J3
                    float(qk_deg_5[3]),  # J4
                    float(qk_deg_5[4]),  # J5
                    float(self.j6_deg),  # J6 fixed
                ]

                self._publish_joints_deg(qk_deg_6)
                time.sleep(self.dt)

            if self.settle_sec > 0.0:
                time.sleep(self.settle_sec)

        self.get_logger().info("Path execution finished.")
        return True

    def _run(self):
        ok = self._run_once()
        if not self.loop:
            return

        while rclpy.ok() and ok:
            ok = self._run_once()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathIKExecutor()
        # 這個 node 自己跑完就結束；不需要 spin
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
