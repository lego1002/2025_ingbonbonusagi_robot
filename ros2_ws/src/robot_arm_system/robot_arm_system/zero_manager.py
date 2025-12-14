#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ev3_interfaces.action import MotorCommand


class ZeroManager(Node):
    def __init__(self):
        super().__init__('zero_manager')

        self.actions = {
            'A': '/ev3A/motorA_action',
            'B': '/ev3A/motorB_action',
            'C': '/ev3A/motorC_action',
            'D': '/ev3B/motorD_action',
            'E': '/ev3B/motorE_action',
            'F': '/ev3B/motorF_action',
        }

        self.motor_id = {
            'A': 1, 'B': 2, 'C': 3,
            'D': 1, 'E': 2, 'F': 3,
        }

        # ❗ 改名，避免撞 Node.clients
        self.action_clients = {
            j: ActionClient(self, MotorCommand, act)
            for j, act in self.actions.items()
        }

        self.wait_for_servers()
        self.reset_all()
        self.go_zero()

        self.get_logger().info('Zero sequence finished.')
        rclpy.shutdown()

    def wait_for_servers(self):
        for c in self.action_clients.values():
            c.wait_for_server()

    def send(self, j, mode, v1=0.0, v2=200.0):
        goal = MotorCommand.Goal()
        goal.mode = mode
        goal.motor_id = self.motor_id[j]
        goal.value1 = float(v1)
        goal.value2 = float(v2)
        return self.action_clients[j].send_goal_async(goal)

    def reset_all(self):
        self.get_logger().info('Reset all motors (encoder zero).')
        futures = [self.send(j, 'reset') for j in self.action_clients]
        rclpy.spin_until_future_complete(self, futures[-1])

    def go_zero(self):
        self.get_logger().info('Go to logical zero (abs 0).')
        futures = [self.send(j, 'abs', 0.0, 150.0) for j in self.action_clients]
        rclpy.spin_until_future_complete(self, futures[-1])


def main():
    rclpy.init()
    ZeroManager()
