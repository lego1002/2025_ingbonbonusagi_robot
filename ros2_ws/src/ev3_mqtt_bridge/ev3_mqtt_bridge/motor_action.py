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
    安全版 Motor Action
    - 啟動時完全不輸出
    - reset 前禁止 hold
    - reset 後才允許 hold
    - 支援 streaming move-like
    """

    def __init__(self):
        super().__init__('motor_action_node')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('ros_cmd_topic', '')
        self.declare_parameter('ros_status_topic', '')
        self.declare_parameter('action_name', 'motor_action')

        # 串流模式：abs/rel/home 發完就 succeed（給 path 用）
        self.declare_parameter('streaming_move_like', True)

        # ❗安全：hold 預設關閉
        self.declare_parameter('hold_enabled', False)
        self.declare_parameter('hold_duty', 8)
        self.declare_parameter('hold_period_sec', 0.4)

        self.cmd_topic = self.get_parameter('ros_cmd_topic').value
        self.status_topic = self.get_parameter('ros_status_topic').value
        self.action_name = self.get_parameter('action_name').value

        self.streaming_move_like = bool(self.get_parameter('streaming_move_like').value)

        self.hold_enabled = bool(self.get_parameter('hold_enabled').value)
        self.hold_duty = int(self.get_parameter('hold_duty').value)
        self.hold_period_sec = float(self.get_parameter('hold_period_sec').value)

        # -------------------------
        # Internal safety state
        # -------------------------
        self._zeroed = False          # ★ reset 後才會變 True
        self._last_cmd_time = time.time()

        # -------------------------
        # Pub / Sub
        # -------------------------
        self.pub = self.create_publisher(String, self.cmd_topic, 10)
        self.sub = self.create_subscription(String, self.status_topic, self.on_status, 10)

        self.last_status = ''
        self.last_status_time = 0.0

        # hold timer（即使有 timer，也會被 _zeroed 擋住）
        self._hold_timer = self.create_timer(self.hold_period_sec, self._hold_tick)

        # -------------------------
        # Action Server
        # -------------------------
        self._server = ActionServer(
            self,
            MotorCommand,
            self.action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        self.get_logger().warn(
            f'[MotorAction] SAFE MODE ENABLED | '
            f'action="{self.get_namespace()}/{self.action_name}", '
            f'hold={"ON" if self.hold_enabled else "OFF"}, '
            f'streaming={"ON" if self.streaming_move_like else "OFF"}'
        )

    # -------------------------
    # Status callback
    # -------------------------
    def on_status(self, msg: String):
        self.last_status = msg.data
        self.last_status_time = time.time()

    # -------------------------
    # Hold logic (SAFE)
    # -------------------------
    def _hold_tick(self):
        # ❗任何一個條件不滿足，就完全不出力
        if not self.hold_enabled:
            return
        if not self._zeroed:
            return
        if (time.time() - self._last_cmd_time) < self.hold_period_sec:
            return

        duty = max(0, min(100, int(self.hold_duty)))
        try:
            self.pub.publish(String(data=f'duty {duty}'))
        except Exception:
            pass

    # -------------------------
    # Action callbacks
    # -------------------------
    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        try:
            self.pub.publish(String(data='stop'))
        except Exception:
            pass
        return CancelResponse.ACCEPT

    # -------------------------
    # Execute
    # -------------------------
    def execute_cb(self, goal_handle):
        req = goal_handle.request
        mode = (req.mode or '').strip().lower()

        try:
            motor_id = int(req.motor_id)
            v1 = float(req.value1)
            v2 = float(req.value2)
        except Exception:
            goal_handle.abort()
            return MotorCommand.Result(success=False, message='Invalid goal parameters')

        cmd = None
        move_deg = 0.0
        move_speed = 0.0

        # -------------------------
        # Parse command
        # -------------------------
        if mode == 'run':
            cmd = 'run'
        elif mode == 'stop':
            cmd = 'stop'
        elif mode == 'reset':
            cmd = 'reset'
            self._zeroed = True   # ★ 關鍵：只有 reset 才算歸零
        elif mode == 'duty':
            duty = max(-100, min(100, int(v1)))
            cmd = f'duty {duty}'
        elif mode in ('rel', 'abs'):
            deg = int(v1)
            speed = int(v2) if int(v2) != 0 else 300
            cmd = f'{mode} {deg} {speed}'
            move_deg = abs(deg)
            move_speed = abs(speed)
        elif mode == 'home':
            speed = int(v1) if int(v1) != 0 else 300
            cmd = f'abs 0 {speed}'
            move_speed = abs(speed)
            move_deg = 1500.0
        else:
            goal_handle.abort()
            return MotorCommand.Result(success=False, message=f'Unknown mode "{mode}"')

        full_cmd = f'm{motor_id}:{cmd}' if motor_id >= 0 else cmd

        # -------------------------
        # SEND
        # -------------------------
        self.pub.publish(String(data=full_cmd))
        self._last_cmd_time = time.time()
        self.get_logger().info(f'[SEND] {full_cmd}')

        # 清掉舊狀態
        command_start_time = time.time()
        self.last_status = ''
        self.last_status_time = 0.0

        # -------------------------
        # Immediate modes
        # -------------------------
        if mode in RUNLIKE:
            goal_handle.succeed()
            return MotorCommand.Result(success=True, message=f'sent: {full_cmd}')

        # -------------------------
        # Streaming mode (KEY)
        # -------------------------
        if self.streaming_move_like and mode in MOVELIKE:
            goal_handle.succeed()
            return MotorCommand.Result(success=True, message=f'stream sent: {full_cmd}')

        # -------------------------
        # Blocking wait (non-streaming)
        # -------------------------
        base_timeout = 5.0
        max_timeout = 30.0

        est_time = (move_deg / move_speed) if (move_speed > 0.0 and move_deg > 0.0) else 0.0
        timeout_sec = min(max_timeout, base_timeout + est_time * 1.5)

        deadline = time.time() + timeout_sec
        STATE_RE = re.compile(r'state\s*=\s*([^,]*)', re.IGNORECASE)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                try:
                    self.pub.publish(String(data='stop'))
                except Exception:
                    pass
                goal_handle.canceled()
                return MotorCommand.Result(success=False, message='Canceled')

            now = time.time()

            if self.last_status and self.last_status_time >= command_start_time:
                m = STATE_RE.search(self.last_status.lower())
                state = m.group(1).strip() if m else ''
                if 'running' not in state:
                    goal_handle.succeed()
                    return MotorCommand.Result(success=True, message='done')

            if now > deadline:
                goal_handle.abort()
                return MotorCommand.Result(success=False, message='timeout')

            time.sleep(0.05)

        goal_handle.abort()
        return MotorCommand.Result(success=False, message='node shutdown')


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
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
