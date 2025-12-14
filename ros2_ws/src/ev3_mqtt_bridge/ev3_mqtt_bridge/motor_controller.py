#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64, Int32
from ev3_interfaces.action import MotorCommand


class MotorController(Node):
    """
    Generic Motor Controller

    Subscribe:
      angle:  /ev3X/motor/motorY_cmd_in   (Float64, joint deg)
      speed:  /ev3X/motor/motorY_speed    (Int32, deg/s)

    Action:
      MotorCommand (abs)
    """

    def __init__(self):
        super().__init__('motor_controller')

        # ===== Parameters =====
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('action_name', '')
        self.declare_parameter('cmd_input_topic', '')
        self.declare_parameter('speed_topic', '')
        self.declare_parameter('ratio', 1.0)
        self.declare_parameter('default_speed', 60.0)

        self.motor_id = int(self.get_parameter('motor_id').value)
        self.action_name = self.get_parameter('action_name').value
        self.cmd_topic = self.get_parameter('cmd_input_topic').value
        self.speed_topic = self.get_parameter('speed_topic').value
        self.ratio = float(self.get_parameter('ratio').value)

        # runtime speed (can change)
        self.speed = float(self.get_parameter('default_speed').value)

        # ===== Action client =====
        self.client = ActionClient(self, MotorCommand, self.action_name)

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Motor action server not available')

        # ===== Subscribers =====
        self.sub_cmd = self.create_subscription(
            Float64,
            self.cmd_topic,
            self.on_cmd,
            10
        )

        if self.speed_topic:
            self.sub_speed = self.create_subscription(
                Int32,
                self.speed_topic,
                self.on_speed,
                10
            )

        self.get_logger().info(
            f'[MotorController] motor_id={self.motor_id}, '
            f'ratio={self.ratio}, default_speed={self.speed}'
        )

    def on_speed(self, msg: Int32):
        self.speed = max(1, int(msg.data))  # safety clamp
        self.get_logger().info(f'[SPEED] set to {self.speed}')

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
            f'[CMD] shaft={shaft_deg:.2f}° → motor={motor_deg:.2f}° '
            f'@ speed={self.speed}'
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
