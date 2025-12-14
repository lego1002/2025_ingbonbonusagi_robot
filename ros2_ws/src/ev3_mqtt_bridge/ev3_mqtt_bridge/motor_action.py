#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import re
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from ev3_interfaces.action import MotorCommand

RUNLIKE = {'run', 'duty', 'stop', 'reset'}
STATE_RE = re.compile(r'state\s*=\s*([^,]*)', re.IGNORECASE)


class MotorAction(Node):
    """
    MotorAction (NO HOLD version)

    行為：
    - 收到 goal → 發一次 abs
    - 等 EV3 回報不在 running → 結束
    - 不做任何持續補償
    """

    def __init__(self):
        super().__init__('motor_action_node')

        self.declare_parameter('ros_cmd_topic', '')
        self.declare_parameter('ros_status_topic', '')
        self.declare_parameter('action_name', 'motor_action')

        self.cmd_topic = self.get_parameter('ros_cmd_topic').value
        self.status_topic = self.get_parameter('ros_status_topic').value
        self.action_name = self.get_parameter('action_name').value

        self.pub = self.create_publisher(String, self.cmd_topic, 10)
        self.sub = self.create_subscription(
            String, self.status_topic, self._on_status, 10
        )

        self.last_status = ''
        self.last_status_time = 0.0

        self._server = ActionServer(
            self,
            MotorCommand,
            self.action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info(
            f'[MotorAction] action="{self.get_namespace()}/{self.action_name}" (NO HOLD)'
        )

    # ---------------- ROS ----------------

    def _on_status(self, msg: String):
        self.last_status = msg.data
        self.last_status_time = time.time()

    # ---------------- Action ----------------

    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        try:
            self.pub.publish(String(data='stop'))
        except Exception:
            pass
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        req = goal_handle.request
        mode = req.mode.strip().lower()

        try:
            motor_id = int(req.motor_id)
            v1 = float(req.value1)
            v2 = float(req.value2)
        except Exception:
            goal_handle.abort()
            return MotorCommand.Result(
                success=False,
                message='Invalid goal parameters'
            )

        # ---- build command ----
        if mode == 'run':
            cmd = 'run'
        elif mode == 'stop':
            cmd = 'stop'
        elif mode == 'reset':
            cmd = 'reset'
        elif mode == 'duty':
            cmd = f'duty {int(v1)}'
        elif mode in ('abs', 'rel'):
            deg = int(v1)
            speed = int(v2) if int(v2) != 0 else 300
            cmd = f'{mode} {deg} {speed}'
        else:
            goal_handle.abort()
            return MotorCommand.Result(
                success=False,
                message=f'Unknown mode "{mode}"'
            )

        full_cmd = f'm{motor_id}:{cmd}'
        self.pub.publish(String(data=full_cmd))
        self.get_logger().info(f'[SEND] {full_cmd}')

        command_time = time.time()
        self.last_status = ''
        self.last_status_time = 0.0

        if mode in RUNLIKE:
            goal_handle.succeed()
            return MotorCommand.Result(success=True, message='sent')

        # ---- wait until not running ----
        timeout = time.time() + 20.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.pub.publish(String(data='stop'))
                goal_handle.canceled()
                return MotorCommand.Result(success=False, message='Canceled')

            if not self.last_status or self.last_status_time < command_time:
                if time.time() > timeout:
                    goal_handle.abort()
                    return MotorCommand.Result(
                        success=False, message='timeout'
                    )
                time.sleep(0.05)
                continue

            m = STATE_RE.search(self.last_status.lower())
            state = m.group(1).strip() if m else ''

            if 'running' not in state:
                goal_handle.succeed()
                return MotorCommand.Result(success=True, message='done')

            if time.time() > timeout:
                goal_handle.abort()
                return MotorCommand.Result(
                    success=False, message='timeout'
                )

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
