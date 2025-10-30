#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ev3_interfaces.srv import MotorCommand

class MotorService(Node):
    """
    Service: /<ns>/motor_service/command
    req: mode, motor_id, value1, value2
    動作：轉為 EV3 指令字串，發佈到 ros_cmd_topic（bridge 會轉到 MQTT）
    """
    def __init__(self) -> None:
        super().__init__('motor_service')

        self.declare_parameter('ros_cmd_topic',   '')
        self.declare_parameter('ros_status_topic','')

        self.cmd_topic: str = self.get_parameter('ros_cmd_topic').get_parameter_value().string_value
        self.status_topic: str = self.get_parameter('ros_status_topic').get_parameter_value().string_value

        self.cmd_pub = self.create_publisher(String, self.cmd_topic, 10)
        self.status_sub = self.create_subscription(String, self.status_topic, self.on_status, 10)

        self.srv = self.create_service(MotorCommand, 'command', self.handle_command)

        self.get_logger().info(f'[MotorService] cmd="{self.cmd_topic}", status="{self.status_topic}"')

    def send_cmd(self, text: str) -> None:
        self.cmd_pub.publish(String(data=text))
        self.get_logger().info(f'[SEND] {text}')

    def on_status(self, msg: String) -> None:
        self.get_logger().info(f'[STATUS] {msg.data}')

    def handle_command(self, req: MotorCommand.Request, res: MotorCommand.Response):
        """
        支援:
          - run / stop / reset
          - duty (value1=duty -100..100)
          - rel  (value1=deg, value2=speed)
          - abs  (value1=deg, value2=speed)
        """
        mode = (req.mode or '').strip().lower()
        motor_id = int(req.motor_id)  # 目前未用；若要多馬達可前綴 m{motor_id}:
        v1 = float(req.value1)
        v2 = float(req.value2)

        cmd = None
        if mode == 'run':
            cmd = 'run'
        elif mode == 'stop':
            cmd = 'stop'
        elif mode == 'reset':
            cmd = 'reset'
        elif mode == 'duty':
            duty = max(-100, min(100, int(v1)))
            cmd = f'duty {duty}'
        elif mode == 'rel':
            deg = int(v1)
            speed = int(v2) if int(v2) != 0 else 300
            cmd = f'rel {deg} {speed}'
        elif mode == 'abs':
            deg = int(v1)
            speed = int(v2) if int(v2) != 0 else 300
            cmd = f'abs {deg} {speed}'
        else:
            res.success = False
            res.message = f'Unknown mode "{mode}"'
            return res

        # 若要支援多馬達：cmd = f'm{motor_id}:{cmd}'
        self.send_cmd(cmd)
        res.success = True
        res.message = f'sent: {cmd}'
        return res

def main(args=None):
    rclpy.init(args=args)
    node = MotorService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
