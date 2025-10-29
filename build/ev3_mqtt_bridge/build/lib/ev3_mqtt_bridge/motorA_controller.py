#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Ev3MotorController(Node):
    def __init__(self):
        super().__init__('ev3_motor_controller')

        # 私域/可參數化 topic
        self.declare_parameter('ros_cmd_topic', 'ev3/motorA_cmd')
        self.declare_parameter('ros_status_topic', 'ev3/motorA_status')

        self.cmd_topic = self.get_parameter('ros_cmd_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('ros_status_topic').get_parameter_value().string_value

        # Publisher / Subscriber
        self.pub = self.create_publisher(String, self.cmd_topic, 10)
        self.sub = self.create_subscription(String, self.status_topic, self.on_status, 10)

        self.get_logger().info(f'[EV3 Motor Controller] cmd="{self.cmd_topic}", status="{self.status_topic}"')

        # 啟動測試序列（延遲 2 秒開始）
        self.step = 0
        self.timer = self.create_timer(0.5, self.sequence)  # 0.5s 檢查一次狀態機
        self.start_time = time.time()
        self.phase_start = None

    def send(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f'[SEND] {text}')

    def on_status(self, msg: String):
        self.get_logger().info(f'[STATUS] {msg.data}')

    def sequence(self):
        # 簡單的狀態機，依序送幾個測試命令
        now = time.time()

        if self.step == 0:
            # 設定 duty 50
            self.send('duty 50')
            self.step = 1
            self.phase_start = now
            return

        if self.step == 1:
            # run 兩秒
            if now - self.phase_start < 0.2:
                self.send('run')
            if now - self.phase_start >= 2.0:
                self.step = 2
                self.phase_start = now
            return

        if self.step == 2:
            self.send('stop')
            self.step = 3
            self.phase_start = now
            return

        if self.step == 3:
            # 正轉 90 度，速度 300 deg/s
            self.send('rel 90 300')
            self.step = 4
            self.phase_start = now
            return

        if self.step == 4:
            if now - self.phase_start >= 2.0:
                # 反轉 90 度
                self.send('rel -90 300')
                self.step = 5
                self.phase_start = now
            return

        if self.step == 5:
            if now - self.phase_start >= 2.0:
                self.send('stop')
                self.get_logger().info('[Sequence] 測試完成')
                self.step = 6
            return

        # step >= 6 不再發指令

def main(args=None):
    rclpy.init(args=args)
    node = Ev3MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
