#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import re
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from ev3_interfaces.action import MotorCommand

RUNLIKE = {'run', 'duty', 'stop', 'reset'}
MOVELIKE = {'rel', 'abs', 'home'}


class MotorAction(Node):
    """
    Motor Action Server with HOLD support
    """

    def __init__(self):
        super().__init__('motor_action_node')

        # ===== Parameters =====
        self.declare_parameter('ros_cmd_topic', '')
        self.declare_parameter('ros_status_topic', '')
        self.declare_parameter('action_name', 'motor_action')

        # HOLD related
        self.declare_parameter('enable_hold', True)
        self.declare_parameter('hold_speed', 50)
        self.declare_parameter('hold_period', 0.5)  # sec

        self.cmd_topic = self.get_parameter('ros_cmd_topic').value
        self.status_topic = self.get_parameter('ros_status_topic').value
        self.action_name = self.get_parameter('action_name').value

        self.enable_hold = self.get_parameter('enable_hold').value
        self.hold_speed = int(self.get_parameter('hold_speed').value)
        self.hold_period = float(self.get_parameter('hold_period').value)

        # ===== Pub/Sub =====
        self.pub = self.create_publisher(String, self.cmd_topic, 10)
        self.sub = self.create_subscription(String, self.status_topic, self.on_status, 10)

        # ===== State =====
        self.last_status = ''
        self.last_status_time = 0.0

        self.last_target_deg = None     # ★ 記住最後目標角度
        self.last_motor_id = None
        self.last_cmd_time = 0.0

        # ===== Action Server =====
        self._server = ActionServer(
            self,
            MotorCommand,
            self.action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        # ===== Hold timer =====
        self.hold_timer = self.create_timer(self.hold_period, self.hold_tick)

        self.get_logger().info(
            f'[MotorAction] action="{self.get_namespace()}/{self.action_name}", '
            f'hold={"ON" if self.enable_hold else "OFF"}'
        )

    # ======================
    # ROS status
    # ======================
    def on_status(self, msg: String):
        self.last_status = msg.data
        self.last_status_time = time.time()

    # ======================
    # Action callbacks
    # ======================
    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        try:
            self.pub.publish(String(data='stop'))
        except Exception:
            pass
        return CancelResponse.ACCEPT

    # ======================
    # HOLD logic
    # ======================
    def hold_tick(self):
        if not self.enable_hold:
            return

        if self.last_target_deg is None or self.last_motor_id is None:
            return

        # 若最近有 command，不補 hold
        if time.time() - self.last_cmd_time < self.hold_period:
            return

        # 若馬達仍在 running，不補 hold
        if 'running' in self.last_status.lower():
            return

        # 補發 hold 指令
        cmd = f'm{self.last_motor_id}:abs {int(self.last_target_deg)} {self.hold_speed}'
        self.pub.publish(String(data=cmd))
        self.get_logger().debug(f'[HOLD] {cmd}')

        self.last_cmd_time = time.time()

    # ======================
    # Execute action
    # ======================
    def execute_cb(self, goal_handle):
        req = goal_handle.request
        mode = (req.mode or '').strip().lower()

        try:
            motor_id = int(req.motor_id)
            v1 = float(req.value1)
            v2 = float(req.value2)
        except Exception:
            goal_handle.abort()
            return MotorCommand.Result(success=False, message='bad params')

        cmd = None
        move_deg = 0.0
        move_speed = 0.0

        if mode == 'run':
            cmd = 'run'
        elif mode == 'stop':
            cmd = 'stop'
        elif mode == 'reset':
            cmd = 'reset'
        elif mode == 'duty':
            cmd = f'duty {int(v1)}'
        elif mode in ('rel', 'abs'):
            deg = int(v1)
            speed = int(v2) if int(v2) != 0 else 300
            cmd = f'{mode} {deg} {speed}'
            move_deg = abs(deg)
            move_speed = abs(speed)

            # ★ 記住目標角度（hold 用）
            if mode == 'abs':
                self.last_target_deg = deg
            elif mode == 'rel' and self.last_target_deg is not None:
                self.last_target_deg += deg

        elif mode == 'home':
            speed = int(v1) if int(v1) != 0 else 300
            cmd = f'abs 0 {speed}'
            self.last_target_deg = 0
            move_deg = 1500.0
            move_speed = abs(speed)
        else:
            goal_handle.abort()
            return MotorCommand.Result(success=False, message='unknown mode')

        full_cmd = f'm{motor_id}:{cmd}'
        self.pub.publish(String(data=full_cmd))
        self.get_logger().info(f'[SEND] {full_cmd}')

        self.last_motor_id = motor_id
        self.last_cmd_time = time.time()
        self.last_status = ''
        self.last_status_time = 0.0

        if mode in RUNLIKE:
            goal_handle.succeed()
            return MotorCommand.Result(success=True, message=f'sent: {full_cmd}')

        # ===== wait for finish =====
        deadline = time.time() + 30.0
        STATE_RE = re.compile(r'state\s*=\s*([^,]*)', re.IGNORECASE)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.pub.publish(String(data='stop'))
                goal_handle.canceled()
                return MotorCommand.Result(success=False, message='canceled')

            if self.last_status and self.last_status_time >= self.last_cmd_time:
                low = self.last_status.lower()
                m = STATE_RE.search(low)
                state = m.group(1).strip() if m else ''
                if 'running' not in state:
                    goal_handle.succeed()
                    return MotorCommand.Result(success=True, message='done')

            if time.time() > deadline:
                goal_handle.abort()
                return MotorCommand.Result(success=False, message='timeout')

            time.sleep(0.05)

        goal_handle.abort()
        return MotorCommand.Result(success=False, message='shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = MotorAction()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
