#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64
from ev3_interfaces.action import MotorCommand


class MotorController(Node):
    """
    Generic Motor Controller (Runtime Only)

    Subscribe:
      /motor/motorX_cmd_in   (Float64, deg)

    Action:
      MotorCommand (abs)
    """

    def __init__(self):
        super().__init__('motor_controller')

        # ===== Parameters =====
        self.declare_parameter('motor_id', 1)          # EV3 port: 1=A,2=B,3=C
        self.declare_parameter('action_name', '')
        self.declare_parameter('cmd_input_topic', '')
        self.declare_parameter('ratio', 1.0)           # shaft_deg * ratio
        self.declare_parameter('speed', 100.0)

        self.motor_id = int(self.get_parameter('motor_id').value)
        self.action_name = self.get_parameter('action_name').value
        self.cmd_topic = self.get_parameter('cmd_input_topic').value
        self.ratio = float(self.get_parameter('ratio').value)
        self.speed = float(self.get_parameter('speed').value)

        # ===== Action client =====
        self.client = ActionClient(self, MotorCommand, self.action_name)

        self.get_logger().info(
            f'[MotorController] motor_id={self.motor_id}, '
            f'action={self.action_name}, ratio={self.ratio}, '
            f'topic={self.cmd_topic}'
        )

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Motor action server not available')

        # ===== Subscriber =====
        self.sub = self.create_subscription(
            Float64,
            self.cmd_topic,
            self.on_cmd,
            10
        )

    def on_cmd(self, msg: Float64):
        shaft_deg = float(msg.data)
        motor_deg = shaft_deg * self.ratio

        goal = MotorCommand.Goal()
        goal.mode = 'abs'
        goal.motor_id = self.motor_id
        goal.value1 = motor_deg
        goal.value2 = self.speed

        self.client.send_goal_async(goal)

        self.get_logger().debug(
            f'[CMD] shaft={shaft_deg:.2f}° → motor={motor_deg:.2f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
