#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class LiquidLevelAdjNode(Node):
    def __init__(self):
        super().__init__('liquid_level_adj_node')

        self.bridge = CvBridge()

        # 訂閱影像
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.get_logger().info("液面標定節點啟動 (使用滑條建立 liquid_level.json)")

        # === 視窗 ===
        cv2.namedWindow("Liquid Level Debug", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Liquid Level Debug", 800, 600)

        cv2.namedWindow("ROI Only", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROI Only", 350, 600)

        cv2.namedWindow("Calibration Panel", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Calibration Panel", 400, 600)

        # ROI 參數
        self.roi_width_ratio = 0.10
        self.roi_top_ratio   = 0.20
        self.roi_bot_ratio   = 0.95

        self.smooth_ksize = 9
        self.blue_offset_roi = None
        self._printed_enc = False

        # 标定 ml 范圍
        self.ml_values = [10,20,30,40,50,60,70,80,90,100]

        # JSON 輸出
        self.json_path = os.path.join(os.getcwd(), "liquid_level.json")

        # 是否已建立滑條
        self.trackbar_ready = False


    # === 建立滑條 ===
    def init_trackbars(self, H_roi, initial_blue):
        if self.trackbar_ready:
            return

        # 藍線滑條
        cv2.createTrackbar("blue_y", "Calibration Panel",
                           initial_blue, H_roi-1, lambda v: None)

        # ml 滑條（預設全部跟藍線一樣）
        for ml in self.ml_values:
            cv2.createTrackbar(f"ml_{ml}", "Calibration Panel",
                               initial_blue, H_roi-1, lambda v: None)

        self.trackbar_ready = True
        self.get_logger().info("滑條建立完成，可開始調整。")


    def image_callback(self, msg: Image):
        try:
            if not self._printed_enc:
                self.get_logger().info(f"來源編碼: {msg.encoding}")
                self._printed_enc = True

            # BGR8
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]

            # ROI 計算
            roi_w = max(4, int(w * self.roi_width_ratio))
            x1 = w//2 - roi_w//2
            x2 = x1 + roi_w
            y1 = int(h * self.roi_top_ratio)
            y2 = int(h * self.roi_bot_ratio)

            y1 = max(0, min(h-2, y1))
            y2 = max(y1+2, min(h, y2))

            roi = frame[y1:y2, x1:x2]
            H_roi = roi.shape[0]
            if H_roi < 5:
                return

            # 自動偵測紅線
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            _, _, V = cv2.split(hsv)
            v_mean = V.mean(axis=1).astype(np.float32)
            v_smooth = cv2.GaussianBlur(v_mean.reshape(-1,1),(1,self.smooth_ksize),0).reshape(-1)
            dv = np.diff(v_smooth)
            margin = max(2, H_roi//20)

            if dv.size > 2*margin:
                dv_mid = dv[margin:-margin]
                idx_red = margin + np.argmax(np.abs(dv_mid))
            else:
                idx_red = np.argmax(np.abs(dv))

            liquid_y = y1 + idx_red

            # 初始化藍線位置
            if self.blue_offset_roi is None:
                self.blue_offset_roi = int(idx_red)

            # 建立滑條（僅一次）
            self.init_trackbars(H_roi, self.blue_offset_roi)

            # 取得滑條值
            blue_offset = cv2.getTrackbarPos("blue_y", "Calibration Panel")
            blue_offset = max(0, min(H_roi-1, blue_offset))
            self.blue_offset_roi = blue_offset
            blue_y = y1 + blue_offset

            # === 顯示主畫面 ===
            debug = frame.copy()
            cv2.rectangle(debug, (x1,y1), (x2,y2), (0,255,255), 1)
            cv2.line(debug, (x1, liquid_y), (x2, liquid_y), (0,0,255), 2)
            cv2.line(debug, (x1, blue_y), (x2, blue_y), (255,0,0), 2)
            cv2.putText(debug, f"Red={liquid_y} Blue={blue_y}",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2)
            cv2.imshow("Liquid Level Debug", debug)

            # ROI 視窗
            roi_vis = roi.copy()
            cv2.line(roi_vis, (0,idx_red), (roi_w,idx_red), (0,0,255),2)
            cv2.line(roi_vis, (0,blue_offset), (roi_w,blue_offset), (255,0,0),2)
            cv2.imshow("ROI Only", roi_vis)

            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.save_json(y1)
            elif key == ord('q'):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"錯誤: {e}")


    # === 儲存 JSON ===
    def save_json(self, roi_y1):
        try:
            data = {}

            for ml in self.ml_values:
                px = cv2.getTrackbarPos(f"ml_{ml}", "Calibration Panel")
                # 換成原圖座標
                data[f"ml_{ml}"] = int(roi_y1 + px)

            with open(self.json_path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)

            self.get_logger().info(f"✅ 已儲存 liquid_level.json：{data}")

        except Exception as e:
            self.get_logger().error(f"JSON 儲存失敗: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LiquidLevelAdjNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
