#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ev3_interfaces.srv import MotorCommand

class MotorAController(Node):
    """
    以 Service Client 呼叫 /<ns>/motor_service/command 做簡單測試流程，
    或訂閱文字 topic，把外部文字命令轉成 service 呼叫。
    """
    def __init__(self) -> None:
        super().__init__('motorA_controller')

        self.declare_parameter('service_ns', '')
        self.declare_parameter('demo_mode', True)
        self.declare_parameter('cmd_input_topic', '')
        self.declare_parameter('motor_id', 0)   # 指定要控制哪一顆（預設 0）

        ns: str     = self.get_parameter('service_ns').get_parameter_value().string_value
        demo: bool  = self.get_parameter('demo_mode').get_parameter_value().bool_value
        self.motor_id: int = int(self.get_parameter('motor_id').get_parameter_value().integer_value)

        # 修正：對應 MotorService 的服務路徑
        srv_name = f'/{ns}/command' if ns else '/command'
        self.cli = self.create_client(MotorCommand, srv_name)

        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f'Waiting for service {srv_name}')

        # demo 狀態機
        self.step = 0
        self.phase_t0 = time.time()
        self.sent_in_step = False  # ★ 每階段只送一次

        if demo:
            self.get_logger().info(f'[MotorAController] demo mode ON (motor_id={self.motor_id})')
            # ★ Timer 改為 0.5s
            self.timer = self.create_timer(0.5, self.demo_tick)
        else:
            topic_in = self.get_parameter('cmd_input_topic').get_parameter_value().string_value
            self.get_logger().info(f'[MotorAController] listen "{topic_in}" and call service (motor_id={self.motor_id})')
            self.sub_in = self.create_subscription(String, topic_in, self.on_text_cmd, 10)

    # --------- 小工具：當前 step 只送一次 ---------
    def send_once_in_step(self, fn):
        if not self.sent_in_step:
            fn()
            self.sent_in_step = True

    def goto_next_step(self):
        self.step += 1
        self.sent_in_step = False
        self.phase_t0 = time.time()

    # --------- demo 流程（每階段只送一次）---------
    def demo_tick(self) -> None:
        now = time.time()
        mid = self.motor_id

        if self.step == 0:
            # duty 10
            self.send_once_in_step(lambda: self.call_cmd('duty', motor_id=mid, v1=40, v2=0))
            # 立即進下一步（不用等）
            self.goto_next_step()

        elif self.step == 1:
            # run（只送一次）
            self.send_once_in_step(lambda: self.call_cmd('run', motor_id=mid))
            # 跑 2 秒
            if now - self.phase_t0 >= 2.0:
                self.goto_next_step()

        elif self.step == 2:
            # stop（只送一次）
            self.send_once_in_step(lambda: self.call_cmd('stop', motor_id=mid))
            # 等 0.2 秒給 EV3 收斂
            if now - self.phase_t0 >= 0.2:
                self.goto_next_step()

        elif self.step == 3:
            # 相對 +90 （只送一次）
            self.send_once_in_step(lambda: self.call_cmd('rel', motor_id=mid, v1=40, v2=300))
            # 等 1.5 秒
            if now - self.phase_t0 >= 1.5:
                self.goto_next_step()

        elif self.step == 4:
            # 相對 -90 （只送一次）
            self.send_once_in_step(lambda: self.call_cmd('rel', motor_id=mid, v1=-40, v2=300))
            # 等 1.5 秒
            if now - self.phase_t0 >= 1.5:
                self.goto_next_step()

        elif self.step == 5:
            # 停止（只送一次）
            self.send_once_in_step(lambda: self.call_cmd('stop', motor_id=mid))
            # 結束
            self.get_logger().info('[Demo] done')
            self.destroy_timer(self.timer)
            self.step = 6

    # --------- 把文字命令轉成 service 呼叫 ---------
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

    # --------- 服務呼叫 helper（非阻塞）---------
    def call_cmd(self, mode: str, motor_id: int = 0, v1: float = 0, v2: float = 0) -> None:
        req = MotorCommand.Request()
        req.mode = str(mode)
        req.motor_id = int(motor_id)
        req.value1 = float(v1)
        req.value2 = float(v2)
        self.cli.call_async(req)   # 不等待，避免阻塞 timer
        self.get_logger().info(f'[CALL] {mode} m{motor_id} ({v1}, {v2})')

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

