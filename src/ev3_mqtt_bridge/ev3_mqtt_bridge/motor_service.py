#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ev3_interfaces.srv import MotorCommand   # ← 這裡改成你自訂的介面

class MotorService(Node):
    """
    單一服務 /<ns>/motor_service/command
    req:  mode, motor_id, value1, value2
    行為：轉成 EV3 能聽懂的命令字串，發佈到 ros_cmd_topic（再由 bridge 送往 MQTT）
    """
    def __init__(self):
        super().__init__('motor_service')

        # 讓 topic 名由參數決定（交給 launch 管）
        self.declare_parameter('ros_cmd_topic',   'motor/motorA_cmd')
        self.declare_parameter('ros_status_topic','motor/motorA_status')

        self.cmd_topic    = self.get_parameter('ros_cmd_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('ros_status_topic').get_parameter_value().string_value

        self.cmd_pub = self.create_publisher(String, self.cmd_topic, 10)
        self.status_sub = self.create_subscription(String, self.status_topic, self.on_status, 10)

        # 單一 MotorCommand 服務
        self.srv = self.create_service(MotorCommand, 'command', self.handle_command)

        self.get_logger().info('[MotorService] cmd="{}", status="{}"'.format(self.cmd_topic, self.status_topic))

    def send_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)
        self.get_logger().info('[SEND] {}'.format(text))

    def on_status(self, msg: String):
        self.get_logger().info('[STATUS] {}'.format(msg.data))

    def handle_command(self, req, res):
        """
        支援的 mode：
          - run / stop / reset
          - duty (value1=duty -100..100)
          - rel  (value1=deg, value2=speed)
          - abs  (value1=deg, value2=speed)  # 先預留
        motor_id: 0..5 或 -1 表示全體（目前範例字串未分馬達，先忽略或加前綴）
        """
        mode = (req.mode or '').strip().lower()
        motor_id = int(req.motor_id)
        v1 = float(req.value1)
        v2 = float(req.value2)

        # 這裡的命令字串先做最簡單版；若要支援多馬達，可加前綴如 "m0:rel 90 300"
        cmd = None
        if mode == 'run':
            cmd = 'run'
        elif mode == 'stop':
            cmd = 'stop'
        elif mode == 'reset':
            cmd = 'reset'
        elif mode == 'duty':
            duty = max(-100, min(100, int(v1)))
            cmd = 'duty {}'.format(duty)
        elif mode == 'rel':
            deg   = int(v1)
            speed = int(v2) if int(v2) != 0 else 300
            cmd = 'rel {} {}'.format(deg, speed)
        elif mode == 'abs':
            deg   = int(v1)
            speed = int(v2) if int(v2) != 0 else 300
            cmd = 'abs {} {}'.format(deg, speed)
        else:
            res.success = False
            res.message = 'Unknown mode "{}"'.format(mode)
            return res

        # 若未來要區分馬達，可在這裡把 motor_id 編進字串
        # 例如：cmd = 'm{}:{}'.format(motor_id, cmd)  # EV3 端再解析

        self.send_cmd(cmd)
        res.success = True
        res.message = 'sent: {}'.format(cmd)
        return res

def main(args=None):
    rclpy.init(args=args)
    node = MotorService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
