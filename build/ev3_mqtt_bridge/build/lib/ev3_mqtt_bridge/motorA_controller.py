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

        ns: str   = self.get_parameter('service_ns').get_parameter_value().string_value
        demo: bool = self.get_parameter('demo_mode').get_parameter_value().bool_value

        srv_name = f'/{ns}/motor_service/command' if ns else '/motor_service/command'
        self.cli = self.create_client(MotorCommand, srv_name)

        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f'Waiting for service {srv_name}')

        if demo:
            self.get_logger().info('[MotorAController] demo mode ON')
            self.step = 0
            self.phase_t0 = time.time()
            self.timer = self.create_timer(0.2, self.demo_tick)
        else:
            topic_in = self.get_parameter('cmd_input_topic').get_parameter_value().string_value
            self.get_logger().info(f'[MotorAController] listen "{topic_in}" and call service')
            self.sub_in = self.create_subscription(String, topic_in, self.on_text_cmd, 10)

    # --------- demo 流程 ---------
    def demo_tick(self) -> None:
        now = time.time()
        if self.step == 0:
            self.call_cmd('duty', motor_id=0, v1=50, v2=0)
            self.phase_t0 = now
            self.step = 1
        elif self.step == 1:
            self.call_cmd('run', motor_id=0)
            if now - self.phase_t0 >= 2.0:
                self.step = 2
        elif self.step == 2:
            self.call_cmd('stop', motor_id=0)
            self.phase_t0 = now
            self.step = 3
        elif self.step == 3:
            self.call_cmd('rel', motor_id=0, v1=90,  v2=300)
            self.phase_t0 = now
            self.step = 4
        elif self.step == 4:
            if now - self.phase_t0 >= 1.5:
                self.call_cmd('rel', motor_id=0, v1=-90, v2=300)
                self.phase_t0 = now
                self.step = 5
        elif self.step == 5:
            if now - self.phase_t0 >= 1.5:
                self.call_cmd('stop', motor_id=0)
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

        try:
            if cmd in ('run', 'stop', 'reset'):
                self.call_cmd(cmd, motor_id=0)
            elif cmd == 'duty' and len(parts) >= 2:
                self.call_cmd('duty', motor_id=0, v1=int(parts[1]), v2=0)
            elif cmd == 'rel' and len(parts) >= 3:
                self.call_cmd('rel', motor_id=0, v1=int(parts[1]), v2=int(parts[2]))
            else:
                self.get_logger().warn(f'Unknown cmd: {txt}')
        except Exception as e:
            self.get_logger().error(f'Cmd error: {e}')

    # --------- 服務呼叫 helper ---------
    def call_cmd(self, mode: str, motor_id: int = 0, v1: float = 0, v2: float = 0) -> None:
        req = MotorCommand.Request()
        req.mode = str(mode)
        req.motor_id = int(motor_id)
        req.value1 = float(v1)
        req.value2 = float(v2)

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        ok = (resp is not None and resp.success)
        self.get_logger().info(f'[CALL] {mode} m{motor_id} ({v1}, {v2}) -> {ok}')

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
