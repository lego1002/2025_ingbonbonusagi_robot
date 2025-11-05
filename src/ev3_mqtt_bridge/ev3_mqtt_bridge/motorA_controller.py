#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ev3_interfaces.srv import MotorCommand

class MotorAController(Node):
    """
    MotorA Controller:
      - demo_mode=True  → 自動跑測試動作 (Duty → Run → Stop → Rel ±90)
      - demo_mode=False → 實際控制模式 (未來可接收 JSON / 指令 topic)
    """

    def __init__(self) -> None:
        super().__init__('motorA_controller')

        # ------------------------
        # 參數宣告區
        # ------------------------
        self.declare_parameter('service_ns', '')
        self.declare_parameter('demo_mode', True)
        self.declare_parameter('cmd_input_topic', '')
        self.declare_parameter('motor_id', 0)   # 指定要控制哪一顆（預設 0）

        ns: str     = self.get_parameter('service_ns').get_parameter_value().string_value
        demo: bool  = self.get_parameter('demo_mode').get_parameter_value().bool_value
        self.motor_id: int = int(self.get_parameter('motor_id').get_parameter_value().integer_value)

        # ------------------------
        # 服務設定
        # ------------------------
        srv_name = f'/{ns}/command' if ns else '/command'
        self.cli = self.create_client(MotorCommand, srv_name)

        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f'Waiting for service {srv_name}')

        # ------------------------
        # 模式切換
        # ------------------------
        if demo:
            self.init_demo_mode()
        else:
            self.init_runtime_mode()

    # =========================================================
    # === DEMO 模式 (demo_mode=True)
    # =========================================================
    def init_demo_mode(self):
        """初始化 Demo 模式的狀態機"""
        self.get_logger().info(f'[MotorAController] DEMO MODE ON (motor_id={self.motor_id})')
        self.step = 0
        self.phase_t0 = time.time()
        self.sent_in_step = False
        # Demo 模式用 timer 定期呼叫
        self.timer = self.create_timer(0.5, self.demo_tick)

    # --------- 小工具：每階段只送一次 ---------
    def send_once_in_step(self, fn):
        if not self.sent_in_step:
            fn()
            self.sent_in_step = True

    def goto_next_step(self):
        self.step += 1
        self.sent_in_step = False
        self.phase_t0 = time.time()

    # --------- Demo 流程 ---------
    def demo_tick(self) -> None:
        now = time.time()
        mid = self.motor_id

        if self.step == 0:
            # duty 40
            self.send_once_in_step(lambda: self.call_cmd('duty', motor_id=mid, v1=40, v2=0))
            self.goto_next_step()

        elif self.step == 1:
            # run
            self.send_once_in_step(lambda: self.call_cmd('run', motor_id=mid))
            if now - self.phase_t0 >= 2.0:
                self.goto_next_step()

        elif self.step == 2:
            # stop
            self.send_once_in_step(lambda: self.call_cmd('stop', motor_id=mid))
            if now - self.phase_t0 >= 0.2:
                self.goto_next_step()

        elif self.step == 3:
            # rel +40
            self.send_once_in_step(lambda: self.call_cmd('rel', motor_id=mid, v1=40, v2=300))
            if now - self.phase_t0 >= 1.5:
                self.goto_next_step()

        elif self.step == 4:
            # rel -40
            self.send_once_in_step(lambda: self.call_cmd('rel', motor_id=mid, v1=-40, v2=300))
            if now - self.phase_t0 >= 1.5:
                self.goto_next_step()

        elif self.step == 5:
            # stop + 結束
            self.send_once_in_step(lambda: self.call_cmd('stop', motor_id=mid))
            self.get_logger().info('[Demo] done')
            self.destroy_timer(self.timer)
            self.step = 6

    # =========================================================
    # === 實際上機模式 (demo_mode=False)
    # =========================================================
    def init_runtime_mode(self):
        """初始化實際控制模式（未來接 JSON 或指令 topic）"""
        topic_in = self.get_parameter('cmd_input_topic').get_parameter_value().string_value
        self.get_logger().info(f'[MotorAController] RUNTIME MODE (listen "{topic_in}" | motor_id={self.motor_id})')

        # 範例：這裡之後可以改為訂閱 JSON 命令或上層規劃節點
        # 現在先保留成文字 topic
        self.sub_in = self.create_subscription(String, topic_in, self.on_text_cmd, 10)

        # TODO:
        # - 未來加入 JSON 解析（如 {"cmd":"rel","value1":90,"value2":300}）
        # - 可支援多 motor_id 控制
        # - 可依照任務序列自動排程

    # =========================================================
    # === 共用功能
    # =========================================================
    def on_text_cmd(self, msg: String) -> None:
        txt = msg.data.strip()
        self.get_logger().info(f'[INPUT] {txt}')
        parts = txt.split()
        if not parts:
            return
        cmd = parts[0].lower()
        mid = self.motor_id

        try:
            if cmd in ('run', 'stop', 'reset'):
                self.call_cmd(cmd, motor_id=mid)
            elif cmd == 'duty' and len(parts) >= 2:
                self.call_cmd('duty', motor_id=mid, v1=int(parts[1]), v2=0)
            elif cmd == 'rel' and len(parts) >= 3:
                self.call_cmd('rel', motor_id=mid, v1=int(parts[1]), v2=int(parts[2]))
            elif cmd == 'abs' and len(parts) >= 3:
                self.call_cmd('abs', motor_id=mid, v1=int(parts[1]), v2=int(parts[2]))
            else:
                self.get_logger().warn(f'Unknown cmd: {txt}')
        except Exception as e:
            self.get_logger().error(f'Cmd error: {e}')

    def call_cmd(self, mode: str, motor_id: int = 0, v1: float = 0, v2: float = 0) -> None:
        req = MotorCommand.Request()
        req.mode = str(mode)
        req.motor_id = int(motor_id)
        req.value1 = float(v1)
        req.value2 = float(v2)
        self.cli.call_async(req)   # 非阻塞
        self.get_logger().info(f'[CALL] {mode} m{motor_id} ({v1}, {v2})')

# =========================================================
# === main
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = MotorAController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
