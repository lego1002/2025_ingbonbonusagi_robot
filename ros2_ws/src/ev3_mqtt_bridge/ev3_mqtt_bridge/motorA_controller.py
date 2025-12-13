#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64
from ev3_interfaces.action import MotorCommand


class MotorAController(Node):
    """
    MotorA Controller (Runtime Only)

    功能：
      - 訂閱 ROS topic: motor/motorA_cmd_in (Float64, deg)
      - 將角度轉成馬達角度 (shaft_deg * ratio)
      - 透過 MotorCommand Action 發送 abs 指令給 EV3
    """

    def __init__(self) -> None:
        super().__init__('motorA_controller')

        # ---------- Parameters ----------
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('action_name', 'motorA_action')
        self.declare_parameter('ratio', 1.0)
        self.declare_parameter('cmd_input_topic', 'motor/motorA_cmd_in')
        self.declare_parameter('speed', 300.0)

        self.motor_id = int(self.get_parameter('motor_id').value)
        self.action_name = self.get_parameter('action_name').value
        self.ratio = float(self.get_parameter('ratio').value)
        self.cmd_topic = self.get_parameter('cmd_input_topic').value
        self.speed = float(self.get_parameter('speed').value)

        # ---------- Action Client ----------
        self.client = ActionClient(self, MotorCommand, self.action_name)

        self.get_logger().info(
            f'[MotorAController] motor_id={self.motor_id}, '
            f'action={self.action_name}, ratio={self.ratio}'
        )

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Motor action server not available')

        # ---------- Subscriber ----------
        self.sub = self.create_subscription(
            Float64,
            self.cmd_topic,
            self.on_angle_cmd,
            10
        )

    def on_angle_cmd(self, msg: Float64):
        """
        接收 IK 發來的「軸角度 (deg)」，轉成馬達角度後送 abs goal
        """
        shaft_deg = float(msg.data)
        motor_deg = shaft_deg * self.ratio

        goal = MotorCommand.Goal()
        goal.mode = 'abs'
        goal.motor_id = self.motor_id
        goal.value1 = motor_deg
        goal.value2 = self.speed

        self.get_logger().debug(
            f'[CMD] shaft={shaft_deg:.2f} deg → motor={motor_deg:.2f} deg'
        )

        self.client.send_goal_async(goal)


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
