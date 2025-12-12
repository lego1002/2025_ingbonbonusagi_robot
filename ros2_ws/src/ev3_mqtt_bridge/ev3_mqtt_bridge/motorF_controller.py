#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from ev3_interfaces.action import MotorCommand


class MotorFController(Node):
    """
    MotorF Controller (Action 版)

    demo_mode=True 時的流程（測試 rel/abs/home 正確性）：
      0) reset                → 把目前位置當作 0
      1) abs  +45 * ratio     → 軸目標 +45 度
      2) rel  -45 * ratio     → 相對 -45 度（應回到 0）
      3) abs  -45 * ratio     → 軸目標 -45 度
      4) rel  +45 * ratio     → 相對 +45 度（應回到 0）
      5) abs  +60 * ratio     → 軸目標 +60 度
      6) home                 → 回到 0（也就是一開始 reset 的位姿）

    ratio 用來把「軸角度」換成「馬達角度」：
      motor_deg = shaft_deg * ratio
    """
    def __init__(self) -> None:
        super().__init__('motorF_controller')

        self.declare_parameter('demo_mode', True)
        self.declare_parameter('cmd_input_topic', '')
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('action_name', 'motorE_action')
        self.declare_parameter('ratio', 1.0)  # ★ 齒輪比（可在 launch 裡調）

        self.demo = self.get_parameter('demo_mode').get_parameter_value().bool_value
        self.motor_id = int(self.get_parameter('motor_id').get_parameter_value().integer_value)
        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.ratio = self.get_parameter('ratio').get_parameter_value().double_value

        # ActionClient 名稱相對於 namespace
        self.client = ActionClient(self, MotorCommand, self.action_name)

        self.get_logger().info(
            f'[MotorFController] connect action: {self.get_namespace()}/{self.action_name}, '
            f'motor_id={self.motor_id}, ratio={self.ratio}'
        )

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Action server not available yet...')

        if self.demo:
            self.init_demo()
        else:
            self.init_runtime()

    # ========== Demo ==========
    def init_demo(self):
        self.get_logger().info(f'[Demo] ON (motor_id={self.motor_id}, ratio={self.ratio})')
        self.step = 0
        self.phase_t0 = time.time()
        self.sent = False
        self.timer = self.create_timer(0.2, self.demo_tick)  # demo tick 稍微快一點也沒關係

    def _deg(self, shaft_deg: float) -> float:
        """把軸角度轉成馬達角度 = 軸角度 * ratio"""
        return float(shaft_deg * self.ratio)

    def send_goal(self, mode, v1=0.0, v2=0.0):
        goal = MotorCommand.Goal()
        goal.mode = str(mode)
        goal.motor_id = int(self.motor_id)
        goal.value1 = float(v1)
        goal.value2 = float(v2)

        self.get_logger().info(f'[GOAL] {mode} m{goal.motor_id} ({goal.value1}, {goal.value2})')

        return self.client.send_goal_async(
            goal,
            feedback_callback=self.on_feedback
        )

    def on_feedback(self, fb):
        self.get_logger().debug(f'[FB] {fb.feedback.status}')

    def demo_tick(self):
        now = time.time()

        # Step 0: reset（把目前位置當成 0）
        if self.step == 0:
            if not self.sent:
                # reset 為立即完成型，不用等 result
                _ = self.send_goal('reset', 0, 0)
                self.sent = True
                self.phase_t0 = now
                self.get_logger().info('[Demo] Step 0: reset (set current pos as 0)')
            else:
                # 稍微等一下，確保 EV3 收到
                if now - self.phase_t0 > 0.3:
                    self.step = 1
                    self.sent = False

        # Step 1: abs +45 * ratio
        elif self.step == 1:
            if not self.sent:
                motor_deg = self._deg(10.0)
                self.goal_future = self.send_goal('abs', motor_deg, 50)
                self.goal_future.add_done_callback(self._on_result)
                self.sent = True
                self.get_logger().info('[Demo] Step 1: abs +45° (x ratio)')
        
        # Step 2: abs -45 * ratio
        elif self.step == 2:
            if not self.sent:
                motor_deg = self._deg(-10.0)
                self.goal_future = self.send_goal('abs', motor_deg, 50)
                self.goal_future.add_done_callback(self._on_result)
                self.sent = True
                self.get_logger().info('[Demo] Step 3: abs -45° (x ratio)')
        
        # Step 3: rel +45 * ratio
        elif self.step == 3:
            if not self.sent:
                motor_deg = self._deg(20)
                self.goal_future = self.send_goal('rel', motor_deg, 50)
                self.goal_future.add_done_callback(self._on_result)
                self.sent = True
                self.get_logger().info('[Demo] Step 4: rel +45° (x ratio) → 應回到 0')

        # Step 4: home（回到 0）
        elif self.step == 4:
            if not self.sent:
                # home: value1 當 speed，用 300；不需要角度
                self.goal_future = self.send_goal('home', 50, 0)
                self.goal_future.add_done_callback(self._on_result)
                self.sent = True
                self.get_logger().info('[Demo] Step 6: home (back to 0)')

        # Step 5: end
        elif self.step == 5:
            if not self.sent:
                self.get_logger().info('[Demo] done (reset → abs/rel test → home)')
                self.destroy_timer(self.timer)
                self.sent = True  # 防止重複 log

    def _on_result(self, goal_handle_future):
        try:
            goal_handle = goal_handle_future.result()
            result_future = goal_handle.get_result_async()

            def _done(rfut):
                result = rfut.result().result
                self.get_logger().info(
                    f'[RESULT] success={result.success}, msg="{result.message}"'
                )
                # 下一步
                self.step += 1
                self.sent = False

            result_future.add_done_callback(_done)
        except Exception as e:
            self.get_logger().error(f'Goal error: {e}')
            self.step += 1
            self.sent = False

    # ========== Runtime (demo_mode=False) ==========
    def init_runtime(self):
        topic_in = self.get_parameter('cmd_input_topic').get_parameter_value().string_value
        self.get_logger().info(
            f'[Runtime] listen "{topic_in}" (motor_id={self.motor_id}, ratio={self.ratio})'
        )
        self.sub = self.create_subscription(String, topic_in, self.on_text_cmd, 10)

    def on_text_cmd(self, msg: String):
        txt = (msg.data or '').strip()
        parts = txt.split()
        if not parts:
            return
        cmd = parts[0].lower()
        try:
            if cmd in ('run', 'stop', 'reset'):
                self.send_goal(cmd)
            elif cmd == 'duty' and len(parts) >= 2:
                self.send_goal('duty', float(parts[1]), 0)
            elif cmd in ('rel', 'abs') and len(parts) >= 3:
                # 這裡假設外部送進來的就是「馬達角度」，不再乘 ratio
                self.send_goal(cmd, float(parts[1]), float(parts[2]))
            elif cmd == 'home':
                self.send_goal('home', 300, 0)
            else:
                self.get_logger().warn(f'Unknown cmd: {txt}')
        except Exception as e:
            self.get_logger().error(f'parse error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()