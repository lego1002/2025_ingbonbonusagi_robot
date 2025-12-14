#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class LiquidLevelNode(Node):
    def __init__(self):
        super().__init__('liquid_level_node')

        self.bridge = CvBridge()

        # ------------------------------
        # ROS2 Parameters
        # ------------------------------
        self.declare_parameter("demo", True)
        self.declare_parameter("target_ml", 50)

        self.demo_mode = self.get_parameter("demo").value
        self.target_ml = self.get_parameter("target_ml").value

        # JSON
        self.json_path = os.path.join(os.getcwd(), "liquid_level.json")
        self.level_table = self.load_json(self.json_path)

        # 若 demo 模式 → 自動抓 JSON 內的像素高度
        if self.demo_mode:
            key = f"ml_{self.target_ml}"
            self.target_y = self.level_table.get(key, None)
            if self.target_y is None:
                self.get_logger().error(f"❌ JSON 中找不到 {key}，請先標定！")
        else:
            # 將來會訂閱來自機械手臂的 liquid_level/height_px
            self.target_y = None

        # ------------------------------
        # 資料來源
        # ------------------------------
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # 將紅線 y 值發布（給 debug 用）
        self.pub_height = self.create_publisher(Int32, "/liquid_level/height_px", 10)

        # 最重要：判斷是否達成目標  → 機械手臂要聽這個
        self.pub_check = self.create_publisher(Bool, "/liquid_level/check", 10)

        self.get_logger().info(f"液面偵測節點啟動 demo={self.demo_mode}, target_ml={self.target_ml}")

        # ------------------------------
        # ROI & 濾波參數
        # ------------------------------
        self.roi_width_ratio = 0.10
        self.roi_top_ratio = 0.20
        self.roi_bot_ratio = 0.95
        self.smooth_ksize = 9

        # ------------------------------
        # 穩定判斷
        # ------------------------------
        self.stable_time = 1  # 秒
        self.last_reach_time = None
        self.reach_state = False  # 最終輸出

        # 視窗
        cv2.namedWindow("Liquid Level Debug", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Liquid Level Debug", 800, 600)

        cv2.namedWindow("ROI Only", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Only", 350, 600)

        self._printed_enc = False


    # ------------------------------
    # 讀取 JSON
    # ------------------------------
    def load_json(self, path):
        if not os.path.exists(path):
            self.get_logger().warn("⚠ 找不到 liquid_level.json")
            return {}

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.get_logger().info(f"已讀取 JSON: {data}")
            return data
        except:
            self.get_logger().error("❌ JSON 解析失敗！")
            return {}


    # ------------------------------
    # 主影像偵測
    # ------------------------------
    def image_callback(self, msg: Image):
        try:
            if not self._printed_enc:
                self.get_logger().info(f"來源編碼: {msg.encoding}")
                self._printed_enc = True

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]

            # ROI
            roi_w = max(4, int(w * self.roi_width_ratio))
            x1 = w//2 - roi_w//2
            x2 = x1 + roi_w

            y1 = int(h * self.roi_top_ratio)
            y2 = int(h * self.roi_bot_ratio)

            roi = frame[y1:y2, x1:x2]
            H_roi = roi.shape[0]
            if H_roi < 5:
                return

            # ------------------------------
            # 亮度曲線 → 液面紅線
            # ------------------------------
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            _, _, V = cv2.split(hsv)

            v_mean = V.mean(axis=1).astype(np.float32)
            v_smooth = cv2.GaussianBlur(v_mean.reshape(-1,1),(1,self.smooth_ksize),0).reshape(-1)

            dv = np.diff(v_smooth)
            margin = max(2, H_roi//20)

            if dv.size > 2*margin:
                dv_mid = dv[margin:-margin]
                off = np.argmax(np.abs(dv_mid))
                idx = margin + off
            else:
                idx = np.argmax(np.abs(dv))

            liquid_y = y1 + idx  # 原圖座標

            # ------------------------------
            # 發布液面高度（debug）
            # ------------------------------
            msg_y = Int32()
            msg_y.data = int(liquid_y)
            self.pub_height.publish(msg_y)

            # ------------------------------
            # 判斷是否到達目標高度
            # ------------------------------
            self.check_target(liquid_y)

            # ------------------------------
            # 顯示畫面
            # ------------------------------
            debug = frame.copy()
            cv2.rectangle(debug, (x1,y1),(x2,y2),(0,255,255),1)
            cv2.line(debug, (x1,liquid_y),(x2,liquid_y),(0,0,255),2)

            # 若 demo 模式 → 顯示目標線
            if self.demo_mode and self.target_y is not None:
                cv2.line(debug, (0,self.target_y),(w,self.target_y),(255,0,0),1)

            cv2.imshow("Liquid Level Debug", debug)

            roi_vis = roi.copy()
            cv2.line(roi_vis, (0,idx),(roi_vis.shape[1],idx),(0,0,255),2)
            cv2.imshow("ROI Only", roi_vis)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"液面偵測失敗: {e}")


    # ------------------------------
    # 判斷液面是否達成目標（含穩定延遲）
    # ------------------------------
    def check_target(self, liquid_y):

        # 若 demo 模式 → 已知目標高度
        if self.demo_mode:
            target_y = self.target_y
        else:
            # 之後會由外部 topic 設定
            if self.target_y is None:
                return
            target_y = self.target_y

        # 判斷是否“到達或超過”目標
        reached_now = (liquid_y <= target_y)

        now = time.time()

        if reached_now:
            # 第一次達標 → 開始計時
            if self.last_reach_time is None:
                self.last_reach_time = now
            else:
                # 已達到穩定秒數
                if now - self.last_reach_time >= self.stable_time:
                    if not self.reach_state:
                        self.get_logger().info("液面達標！")
                    self.reach_state = True
        else:
            # 一旦不達標 → 重置
            self.last_reach_time = None
            if self.reach_state:
                self.get_logger().info("液面離開目標區間")
            self.reach_state = False

        # Publish 結果
        msg = Bool()
        msg.data = self.reach_state
        self.pub_check.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LiquidLevelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
