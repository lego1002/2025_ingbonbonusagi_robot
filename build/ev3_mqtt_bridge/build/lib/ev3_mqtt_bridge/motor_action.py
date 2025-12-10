#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import re
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from ev3_interfaces.action import MotorCommand  # Action 介面

RUNLIKE = {'run', 'duty', 'stop', 'reset'}   # 立即完成型
MOVELIKE = {'rel', 'abs', 'home'}            # 需等待完成型


class MotorAction(Node):
    """
    Action: <namespace>/<action_name>
    Goal: mode, motor_id, value1, value2
    Result: success, message
    Feedback: status (EV3 原樣字串)
    """
    def __init__(self):
        # node name 改成比較不容易跟 action 名撞在一起
        super().__init__('motor_action_node')

        # 參數（由 launch 注入）
        self.declare_parameter('ros_cmd_topic', '')
        self.declare_parameter('ros_status_topic', '')
        self.declare_parameter('action_name', 'motor_action')  # 預設 action 名稱

        self.cmd_topic: str = self.get_parameter('ros_cmd_topic').get_parameter_value().string_value
        self.status_topic: str = self.get_parameter('ros_status_topic').get_parameter_value().string_value
        self.action_name: str = self.get_parameter('action_name').get_parameter_value().string_value

        # Pub/Sub
        self.pub = self.create_publisher(String, self.cmd_topic, 10)
        self.sub = self.create_subscription(String, self.status_topic, self.on_status, 10)

        # 近期一筆狀態
        self.last_status = ''
        self.last_status_time = 0.0

        # Action Server
        self._server = ActionServer(
            self,
            MotorCommand,             # ← Action 型別是 ev3_interfaces.action.MotorCommand
            self.action_name,         # ← 用參數決定 action 名稱
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        self.get_logger().info(
            f'[MotorAction] action="{self.get_namespace()}/{self.action_name}", '
            f'cmd="{self.cmd_topic}", status="{self.status_topic}"'
        )

    # ============ ROS 狀態 ============
    def on_status(self, msg: String):
        self.last_status = msg.data
        self.last_status_time = time.time()
        # 可暫時開 debug 看是否有收到
        # self.get_logger().info(f'[STATUS] {self.last_status}')

    # ============ Action Callbacks ============
    def goal_cb(self, goal_request):
        # 全部接受
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        # 支援取消（Move 型可在這裡發 stop）
        try:
            self.pub.publish(String(data='stop'))
        except Exception:
            pass
        return CancelResponse.ACCEPT

    # ============ 執行動作（同步版本） ============
    def execute_cb(self, goal_handle):
        req = goal_handle.request
        mode = (req.mode or '').strip().lower()

        try:
            motor_id = int(req.motor_id)
            v1 = float(req.value1)
            v2 = float(req.value2)
        except Exception:
            goal_handle.abort()
            return MotorCommand.Result(
                success=False,
                message='motor_id/value1/value2 格式錯誤'
            )

        # 組指令字串 + 為動態 timeout 準備移動角度/速度
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
            duty = max(-100, min(100, int(v1)))
            cmd = f'duty {duty}'
        elif mode in ('rel', 'abs'):
            deg = int(v1)
            speed = int(v2) if int(v2) != 0 else 300
            cmd = f'{mode} {deg} {speed}'
            move_deg = abs(deg)
            move_speed = abs(speed)
        elif mode == 'home':
            # 讓它走回 0 度；v1 可當 speed 參數，不給就用 300
            speed = int(v1) if int(v1) != 0 else 300
            cmd = f'abs 0 {speed}'
            move_speed = abs(speed)
            # 這裡很難從指令推真正角度，先抓一個上限估計（例如一圈多）
            # 之後如果你有「目前位置」資訊，也可以從狀態推更準確的 move_deg
            move_deg = 1500.0
        else:
            goal_handle.abort()
            return MotorCommand.Result(success=False, message=f'Unknown mode "{mode}"')

        full_cmd = f'm{motor_id}:{cmd}' if motor_id >= 0 else cmd
        self.pub.publish(String(data=full_cmd))
        self.get_logger().info(f'[SEND] {full_cmd}')

        # 發完指令後記錄時間 & 清掉舊狀態，避免吃到前一個命令的 reset / heartbeat
        command_start_time = time.time()
        self.last_status = ''
        self.last_status_time = 0.0

        # 立即型：直接成功
        if mode in RUNLIKE:
            goal_handle.succeed()
            return MotorCommand.Result(success=True, message=f'sent: {full_cmd}')

        # ======================
        # 移動型：估計 timeout 時間
        # ======================
        base_timeout = 5.0      # 無論如何至少等 5 秒
        max_timeout = 30.0      # 上限，避免失控等太久

        if move_speed > 0.0 and move_deg > 0.0:
            est_time = move_deg / move_speed
        else:
            est_time = 0.0

        # 估計時間 * 1.5 加安全係數
        timeout_sec = base_timeout + est_time * 1.5
        if timeout_sec > max_timeout:
            timeout_sec = max_timeout

        deadline = time.time() + timeout_sec
        self.get_logger().info(
            f'[MotorAction] mode={mode}, move_deg={move_deg}, '
            f'move_speed={move_speed}, timeout={timeout_sec:.2f}s'
        )

        # ======================
        # 等待完成或逾時/取消
        # ======================
        last_feedback_sent = 0.0
        STATE_RE = re.compile(r'state\s*=\s*([^,]*)', re.IGNORECASE)

        while rclpy.ok():
            # 取消請求
            if goal_handle.is_cancel_requested:
                try:
                    self.pub.publish(String(data='stop'))
                except Exception:
                    pass
                goal_handle.canceled()
                return MotorCommand.Result(success=False, message='Canceled')

            now = time.time()

            # 送 feedback（節流 0.2s）
            if self.last_status and now - last_feedback_sent > 0.2:
                fb = MotorCommand.Feedback()
                fb.status = self.last_status
                goal_handle.publish_feedback(fb)
                last_feedback_sent = now

            # 沒有新狀態 或 狀態太舊（在這個 command 之前的） → 等下一輪
            if (not self.last_status) or (self.last_status_time < command_start_time):
                if now > deadline:
                    goal_handle.abort()
                    return MotorCommand.Result(success=False, message='timeout (no status)')
                self._sleep(0.05)
                continue

            low = self.last_status.lower()

            # 解析 state=
            m_state = STATE_RE.search(low)
            state_str = m_state.group(1).strip() if m_state else ''

            moving = ('running' in state_str)

            if not moving:
                # 不在 running 狀態 → 視為完成
                goal_handle.succeed()
                return MotorCommand.Result(
                    success=True,
                    message=f'done: {full_cmd} | status="{self.last_status}"'
                )

            # 還在 running：持續等
            if now > deadline:
                goal_handle.abort()
                return MotorCommand.Result(success=False, message='timeout (still running)')

            self._sleep(0.05)

        # Node 被 shutdown
        goal_handle.abort()
        return MotorCommand.Result(success=False, message='node shutdown')

    def _sleep(self, sec: float):
        # 這裡直接 time.sleep 即可，ActionServer 會在自己的 thread 跑，
        # MultiThreadedExecutor 會讓其他 callback 在別的 thread 執行
        time.sleep(sec)


def main(args=None):
    rclpy.init(args=args)
    node = MotorAction()

    # ★ 用 MultiThreadedExecutor，避免 execute_cb() 的 while loop 卡住訂閱 callback
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

