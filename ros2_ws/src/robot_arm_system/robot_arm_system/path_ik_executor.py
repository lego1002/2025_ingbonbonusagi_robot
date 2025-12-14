#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from ev3_interfaces.action import MotorCommand

# ⚠️ IK 不動：照你原本的運動學算法
from .ik_test import (
    ik_constrained,
    check_joint_limits,
    q_lims,
    DEG
)


# ============================
# Utils
# ============================

def smoothstep(s: float) -> float:
    # 0~1 -> 0~1, with zero slope at ends (C1 smooth)
    return s * s * (3.0 - 2.0 * s)


class JointTrajectoryPlanner:
    """
    q0->q1 生成固定 dt 的平滑 setpoint（關節空間）
    注意：這裡只做「小步」的平滑，真正安全靠 Cartesian 插值 + 每點 IK
    """
    def __init__(self, dt: float = 0.03, v_max_deg_s: float = 25.0):
        self.dt = float(dt)
        self.v_max = float(v_max_deg_s)

    def plan(self, q0_deg: np.ndarray, q1_deg: np.ndarray):
        q0_deg = np.asarray(q0_deg, dtype=float).reshape(-1)
        q1_deg = np.asarray(q1_deg, dtype=float).reshape(-1)
        dq = q1_deg - q0_deg

        max_dq = float(np.max(np.abs(dq)))
        if max_dq < 1e-9:
            return [q1_deg.copy()]

        # 以最大關節位移決定段落時間，確保不超速
        T = max(self.dt, max_dq / max(1e-6, self.v_max))
        steps = max(1, int(np.ceil(T / self.dt)))

        traj = []
        for i in range(1, steps + 1):
            s = i / steps
            ss = smoothstep(s)
            q = q0_deg + ss * dq
            traj.append(q)

        return traj


def interpolate_xyz(p0: np.ndarray, p1: np.ndarray, step_m: float):
    """
    Cartesian 線性插值，回傳 list[np.array(3,)]
    step_m: 每步距離（公尺）
    """
    p0 = np.asarray(p0, dtype=float).reshape(3)
    p1 = np.asarray(p1, dtype=float).reshape(3)
    d = float(np.linalg.norm(p1 - p0))
    if d < 1e-12:
        return [p1.copy()]

    step_m = max(1e-6, float(step_m))
    n = max(1, int(np.ceil(d / step_m)))

    out = []
    for i in range(1, n + 1):
        s = i / n
        p = (1.0 - s) * p0 + s * p1
        out.append(p)
    return out


# ============================
# Main executor
# ============================

class PathIKExecutor(Node):
    def __init__(self):
        super().__init__('path_ik_executor')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter(
            'json_path',
            '/home/lego/Desktop/2025_ingbonbonusagi_robot/ros2_ws/test_point.json'
        )

        # Approach/Retreat（避開低點/障礙用）
        self.declare_parameter('z_safe', 0.20)
        self.declare_parameter('always_use_approach', True)
        self.declare_parameter('retreat_after_each_point', True)

        # ✅ 安全版核心：Cartesian 插值步距（越小越貼近直線、越安全、越耗算）
        self.declare_parameter('cart_step_m', 0.01)   # 1cm，建議先用 0.01，確認穩再降到 0.005

        # 關節平滑（在 Cartesian 已安全的前提下，只用來變順）
        self.declare_parameter('dt', 0.03)            # setpoint 更新周期
        self.declare_parameter('v_max_deg_s', 25.0)   # 關節速度上限（deg/s）

        # Motor
        self.declare_parameter('speed', 120.0)        # 給 motor_action 的 speed 參數
        self.declare_parameter('use_motor_f', False)  # IK 只有 5 軸，F 先不動

        # ✅ 工作空間限制（你說「超出手臂運動空間」就靠這個擋）
        # 以 base 為中心的半徑 r = sqrt(x^2+y^2)
        self.declare_parameter('workspace_r_min', 0.00)
        self.declare_parameter('workspace_r_max', 0.35)
        self.declare_parameter('workspace_z_min', 0.00)
        self.declare_parameter('workspace_z_max', 0.40)

        # Debug
        self.declare_parameter('log_every_n_setpoints', 80)

        self.json_path = str(self.get_parameter('json_path').value)

        self.z_safe = float(self.get_parameter('z_safe').value)
        self.always_use_approach = bool(self.get_parameter('always_use_approach').value)
        self.retreat_after_each_point = bool(self.get_parameter('retreat_after_each_point').value)

        self.cart_step_m = float(self.get_parameter('cart_step_m').value)

        self.dt = float(self.get_parameter('dt').value)
        self.v_max_deg_s = float(self.get_parameter('v_max_deg_s').value)

        self.speed = float(self.get_parameter('speed').value)
        self.use_motor_f = bool(self.get_parameter('use_motor_f').value)

        self.ws_r_min = float(self.get_parameter('workspace_r_min').value)
        self.ws_r_max = float(self.get_parameter('workspace_r_max').value)
        self.ws_z_min = float(self.get_parameter('workspace_z_min').value)
        self.ws_z_max = float(self.get_parameter('workspace_z_max').value)

        self.log_every_n = int(self.get_parameter('log_every_n_setpoints').value)

        # -------------------------
        # Action servers (A~F)
        # -------------------------
        self.action_names = {
            'A': '/ev3A/motorA_action',
            'B': '/ev3A/motorB_action',
            'C': '/ev3A/motorC_action',
            'D': '/ev3B/motorD_action',
            'E': '/ev3B/motorE_action',
            'F': '/ev3B/motorF_action',
        }
        # EV3 ports: 1=A,2=B,3=C
        self.motor_id = {'A': 1, 'B': 2, 'C': 3, 'D': 1, 'E': 2, 'F': 3}

        # ⚠️ 不要用 self.clients（Node 裡可能撞到），用 action_clients
        self.action_clients = {
            j: ActionClient(self, MotorCommand, name)
            for j, name in self.action_names.items()
        }

        # RViz joint state publisher
        self.js_pub = self.create_publisher(JointState, '/ik_joint_states', 10)

        # 執行一次就結束
        self._done = False
        self.create_timer(0.1, self._tick_once)

    # -------------------------
    # One-shot tick
    # -------------------------
    def _tick_once(self):
        if self._done:
            return
        self._done = True

        # 1) 讀 JSON keypoints
        key_xyz = self.load_keypoints_xyz()
        self.get_logger().info(f'Loaded {len(key_xyz)} keypoints from JSON.')

        # 2) 生成 approach/retreat exec points
        exec_xyz = self.expand_with_approach_retreat(key_xyz)
        self.get_logger().info(f'Expanded to {len(exec_xyz)} exec points (with approach/retreat).')

        # 3) 再把 exec_xyz 做 Cartesian 插值（安全直線路徑）
        path_xyz = self.build_cartesian_path(exec_xyz, self.cart_step_m)
        self.get_logger().info(f'Cartesian interpolation: step={self.cart_step_m:.4f} m, total_xyz_points={len(path_xyz)}')

        # 4) 等 action server
        self.wait_for_servers()

        # 5) 每個 XYZ 做 IK + joint limit + workspace 檢查，得到 q_path
        q_path = self.solve_path_ik(path_xyz)
        if q_path is None or len(q_path) < 2:
            self.get_logger().error('IK path empty / failed.')
            return

        self.get_logger().info(f'IK path ready: q_points={len(q_path)}')

        # 6) 串流執行（安全路徑已在 XYZ 層做完）
        self.stream_execute(q_path)

        self.get_logger().info('Path execution finished.')

        # 不在這裡 rclpy.shutdown()（避免跟 launch/ctrl-c 互相踩）
        # main() 會在 done 後安全結束 spin

    # -------------------------
    # JSON loader
    # -------------------------
    def load_keypoints_xyz(self) -> np.ndarray:
        with open(self.json_path, 'r') as f:
            raw = json.load(f)

        # 支援 {"0":[...], "1":[...]} 形式
        if isinstance(raw, dict):
            pts = [raw[k] for k in sorted(raw.keys(), key=int)]
        elif isinstance(raw, list):
            # 也容許 list of {"x":..,"y":..,"z":..}
            pts = []
            for it in raw:
                if isinstance(it, dict) and all(k in it for k in ['x', 'y', 'z']):
                    pts.append([it['x'], it['y'], it['z']])
                else:
                    pts.append(it)
        else:
            raise RuntimeError('Unsupported JSON format for keypoints.')

        pts = np.array(pts, dtype=float).reshape(-1, 3)  # meters
        return pts

    # -------------------------
    # Approach/Retreat expansion
    # -------------------------
    def expand_with_approach_retreat(self, pts_xyz: np.ndarray) -> np.ndarray:
        if pts_xyz.shape[0] == 0:
            return pts_xyz

        z_safe = self.z_safe
        out = []

        # 第一點：先拉到安全高度
        p0 = pts_xyz[0].copy()
        p0_hi = p0.copy()
        p0_hi[2] = max(z_safe, p0[2])
        out.append(p0_hi)

        if self.always_use_approach and p0[2] < p0_hi[2] - 1e-9:
            out.append(p0.copy())

        for i in range(1, pts_xyz.shape[0]):
            p = pts_xyz[i].copy()
            p_hi = p.copy()
            p_hi[2] = max(z_safe, p[2])

            prev = out[-1].copy()

            # 離開上一點前也先回安全高度（可選）
            if self.always_use_approach:
                prev_hi = prev.copy()
                prev_hi[2] = max(z_safe, prev[2])
                if np.linalg.norm(prev_hi - prev) > 1e-9:
                    out.append(prev_hi)

            # 平移到目標上方
            if np.linalg.norm(p_hi - out[-1]) > 1e-9:
                out.append(p_hi)

            # 下降到目標
            if p[2] < p_hi[2] - 1e-9:
                if np.linalg.norm(p - out[-1]) > 1e-9:
                    out.append(p)

            # 到點後撤回安全高度
            if self.retreat_after_each_point:
                if np.linalg.norm(p_hi - out[-1]) > 1e-9:
                    out.append(p_hi)

        # 去掉重複
        compact = [out[0]]
        for p in out[1:]:
            if np.linalg.norm(np.asarray(p) - np.asarray(compact[-1])) > 1e-9:
                compact.append(p)

        return np.array(compact, dtype=float)

    # -------------------------
    # Cartesian path builder
    # -------------------------
    def build_cartesian_path(self, exec_xyz: np.ndarray, step_m: float) -> np.ndarray:
        if exec_xyz.shape[0] == 0:
            return exec_xyz

        out = [exec_xyz[0].copy()]
        for i in range(exec_xyz.shape[0] - 1):
            seg = interpolate_xyz(exec_xyz[i], exec_xyz[i + 1], step_m)
            out.extend(seg)
        return np.array(out, dtype=float)

    # -------------------------
    # Workspace constraint
    # -------------------------
    def within_workspace(self, xyz: np.ndarray) -> bool:
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        r = (x * x + y * y) ** 0.5
        if r < self.ws_r_min - 1e-12:
            return False
        if r > self.ws_r_max + 1e-12:
            return False
        if z < self.ws_z_min - 1e-12:
            return False
        if z > self.ws_z_max + 1e-12:
            return False
        return True

    # -------------------------
    # Action helpers
    # -------------------------
    def wait_for_servers(self):
        self.get_logger().info('Waiting for all motor action servers...')
        for c in self.action_clients.values():
            c.wait_for_server()
        self.get_logger().info('All action servers ready.')

    def publish_joint_state(self, q_rad_5):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        msg.position = list(q_rad_5)
        self.js_pub.publish(msg)

    def send_abs_stream(self, joint_letter: str, joint_deg: float, speed: float):
        """
        串流送 abs setpoint（不等待完成）
        需搭配 motor_action 的 streaming_move_like=True
        """
        goal = MotorCommand.Goal()
        goal.mode = 'abs'
        goal.motor_id = int(self.motor_id[joint_letter])
        goal.value1 = float(joint_deg)
        goal.value2 = float(speed)
        self.action_clients[joint_letter].send_goal_async(goal)

    # -------------------------
    # IK for every XYZ point
    # -------------------------
    def solve_path_ik(self, path_xyz: np.ndarray):
        q_path = []
        q_seed = np.zeros(5)

        for i, xyz in enumerate(path_xyz):
            if not self.within_workspace(xyz):
                r = float(np.hypot(xyz[0], xyz[1]))
                self.get_logger().error(
                    f'Workspace violation at xyz[{i}]={xyz.tolist()} r={r:.3f} '
                    f'(r[{self.ws_r_min:.3f},{self.ws_r_max:.3f}], z[{self.ws_z_min:.3f},{self.ws_z_max:.3f}])'
                )
                return None

            q_sol, err, ok = ik_constrained(xyz, q_seed)
            if not ok:
                self.get_logger().error(f'IK failed at xyz[{i}]={xyz.tolist()} err={err:.6f}')
                return None

            violations = check_joint_limits(q_sol, q_lims)
            if violations:
                self.get_logger().error(f'Joint limit violation at xyz[{i}]={xyz.tolist()}')
                return None

            q_path.append(q_sol.copy())
            q_seed = q_sol.copy()

            if i == 0 or (i % 300 == 0):
                self.get_logger().info(f'IK progress {i+1}/{len(path_xyz)}')

        return np.array(q_path, dtype=float)

    # -------------------------
    # Execute with smooth streaming
    # -------------------------
    def stream_execute(self, q_path: np.ndarray):
        planner = JointTrajectoryPlanner(dt=self.dt, v_max_deg_s=self.v_max_deg_s)

        # 估算 setpoints
        est = 0
        for i in range(len(q_path) - 1):
            est += len(planner.plan(q_path[i] / DEG, q_path[i + 1] / DEG))

        self.get_logger().info(
            f'Streaming execute: dt={self.dt:.3f}s, v_max={self.v_max_deg_s:.1f} deg/s, setpoints≈{est}'
        )

        t_next = time.time()
        sent = 0

        for i in range(len(q_path) - 1):
            q0_deg = q_path[i] / DEG
            q1_deg = q_path[i + 1] / DEG
            traj = planner.plan(q0_deg, q1_deg)

            for q_deg in traj:
                sent += 1
                q_rad = q_deg * DEG

                # RViz
                self.publish_joint_state(q_rad)

                # Log
                if sent == 1 or (self.log_every_n > 0 and sent % self.log_every_n == 0) or sent == est:
                    q_print = [round(float(v), 1) for v in q_deg.tolist()]
                    self.get_logger().info(f'Setpoint {sent}/{est} q_deg={q_print}')

                # 送馬達 A~E（對應 joint1~joint5）
                for j_letter, val_deg in zip(['A', 'B', 'C', 'D', 'E'], q_deg):
                    self.send_abs_stream(j_letter, float(val_deg), self.speed)

                if self.use_motor_f:
                    self.send_abs_stream('F', 0.0, self.speed)

                # 固定 dt 節奏（避免漂移）
                now = time.time()
                if now < t_next:
                    time.sleep(t_next - now)
                t_next += self.dt


def main():
    rclpy.init()
    node = PathIKExecutor()

    try:
        # spin 到 node 真的跑完
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if getattr(node, '_done', False):
                # 已執行完畢，給一點時間讓最後幾個 async goal 送出
                time.sleep(0.2)
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
