#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import paho.mqtt.client as mqtt
import json
import numpy as np
import threading
import sys

# 機器人數學庫
import roboticstoolbox as rtb
from spatialmath import SE3

class LegoArmController(Node):
    def __init__(self):
        super().__init__('lego_arm_ik_controller')

        # ---------------------------------------------------------
        # 1. 建立機器人模型 (移植自 MATLAB: lego_final_orientation_fix.m)
        # ---------------------------------------------------------
        # d=偏移, a=長度, alpha=扭轉, offset=初始角度偏移
        deg = np.pi / 180.0
        mm = 0.001
        
        # 定義連桿 (Standard DH)
        L1 = rtb.RevoluteDH(d=105*mm, a=23*mm, alpha=90*deg)
        L2 = rtb.RevoluteDH(d=0,      a=135*mm, alpha=0,       offset=90*deg)
        L3 = rtb.RevoluteDH(d=0,      a=35*mm,  alpha=-90*deg)
        L4 = rtb.RevoluteDH(d=137*mm, a=0,      alpha=90*deg,  offset=-90*deg)
        L5 = rtb.RevoluteDH(d=0,      a=97*mm,  alpha=0,       offset=-90*deg)

        self.robot = rtb.DHRobot([L1, L2, L3, L4, L5], name="LegoArm")
        self.get_logger().info(f"機器人模型建立完成:\n{self.robot}")

        # ---------------------------------------------------------
        # 2. 設定 MQTT (連接 EV3)
        # ---------------------------------------------------------
        self.mqtt_broker = "192.168.0.x"  # ★★★ 請修改成您電腦或 EV3 的 IP ★★★
        self.mqtt_topic = "lego/motor_cmd" # ★★★ 請確認這跟您 EV3 聽的 Topic 一樣 ★★★
        self.client = mqtt.Client()
        
        try:
            self.client.connect(self.mqtt_broker, 1883, 60)
            self.client.loop_start()
            self.get_logger().info(f"MQTT 已連接至 {self.mqtt_broker}")
        except Exception as e:
            self.get_logger().warn(f"MQTT 連接失敗 (模擬模式): {e}")

        # ---------------------------------------------------------
        # 3. 設定 ROS2 Publisher (連接 Rviz)
        # ---------------------------------------------------------
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 初始姿勢 (Ready Pose)
        self.q_curr = np.array([0, 10, -10, 0, 0]) * deg
        self.publish_joints(self.q_curr)

        # 啟動互動輸入執行緒
        threading.Thread(target=self.input_loop, daemon=True).start()

    def input_loop(self):
        """ 終端機互動介面 """
        print("\n===========================================")
        print("   LEGO 5-Axis Arm - 座標控制系統 (IK)   ")
        print("===========================================")
        print("請輸入目標座標 (單位 mm)，例如: 200 0 150")
        print("輸入 'q' 離開")
        
        while rclpy.ok():
            try:
                user_in = input("\n輸入目標 (x y z) >> ")
                if user_in.lower() == 'q':
                    rclpy.shutdown()
                    sys.exit(0)
                
                parts = user_in.split()
                if len(parts) != 3:
                    print("格式錯誤！請輸入三個數字，例如: 150 50 200")
                    continue

                x, y, z = map(float, parts)
                self.move_to_point(x, y, z)

            except ValueError:
                print("請輸入有效的數字！")
            except Exception as e:
                print(f"錯誤: {e}")

    def move_to_point(self, x_mm, y_mm, z_mm):
        """ 核心: 逆向運動學解算與控制 """
        
        # 1. 定義目標位置 (將 mm 轉為 m)
        target_pos = [x_mm * 0.001, y_mm * 0.001, z_mm * 0.001]
        
        # 2. 建構目標姿態矩陣 (SE3)
        # 因為是 5 軸，我們主要關注位置。姿態部分我們先假設夾爪「水平向前」
        # SE3.Trans 建立平移，SE3.RPY 建立旋轉 (這裡設為朝前)
        T_target = SE3.Trans(target_pos) * SE3.RPY(0, 0, 0)

        print(f"計算 IK 目標: {target_pos} m")

        # 3. 執行 IK 解算 (Levenberg-Marquardt 數值解法)
        # mask=[1,1,1,1,1,0] 代表我們重視 x,y,z,rx,ry，忽略 rz (由結構決定)
        # q0=self.q_curr: 從當前角度開始算，收斂較快且較安全
        sol = self.robot.ikine_LM(T_target, q0=self.q_curr, mask=[1, 1, 1, 1, 1, 0])

        if sol.success:
            print("✅ IK 解算成功！")
            q_new = sol.q
            
            # 顯示角度 (轉成度數給人看)
            q_deg = np.round(q_new * 180 / np.pi, 2)
            print(f"關節角度: {q_deg}")

            # 4. 更新狀態
            self.q_curr = q_new
            self.publish_joints(q_new)      # 動 Rviz
            self.send_mqtt_command(q_deg)   # 動 EV3 (真手)
        else:
            print("❌ IK 解算失敗 (目標可能超出工作範圍)")

    def publish_joints(self, q):
        """ 發送 JointState 給 Rviz """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        msg.position = q.tolist() # ROS 使用弧度 (rad)
        self.joint_pub.publish(msg)

    def send_mqtt_command(self, q_deg):
        """ 發送命令給 EV3 (轉成 JSON) """
        # 假設您的 EV3 接收格式是 {"J1": 角度, "J2": 角度 ...}
        payload = {
            "target_angles": {
                "J1": float(q_deg[0]),
                "J2": float(q_deg[1]),
                "J3": float(q_deg[2]),
                "J4": float(q_deg[3]),
                "J5": float(q_deg[4])
            },
            "speed": 20  # 您可以加入速度控制
        }
        try:
            self.client.publish(self.mqtt_topic, json.dumps(payload))
            print(f"📡 MQTT 命令已發送: {payload}")
        except Exception as e:
            print(f"MQTT 發送失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LegoArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()