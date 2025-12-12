#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class SobelEdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('sobel_edge_detection_node')

        self.bridge = CvBridge()

        # 訂閱 /camera/image_raw（相機原始影像）
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        # 發佈邊緣圖
        self.pub = self.create_publisher(Image, '/camera/image_edge', 10)

        self.get_logger().info("Sobel邊緣檢測節點已啟動 (訂閱 /camera/image_raw)")

        # 視窗
        cv2.namedWindow("Sobel Edge Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Sobel Edge Detection", 800, 600)
        cv2.namedWindow("CLAHE Enhanced Image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("CLAHE Enhanced Image", 800, 600)

        # ROI 形狀：中心長方形（高大於寬）
        # 使用畫面尺寸比例來定義：例如 寬度 30% 、高度 80%
        self.roi_width_ratio = 0.3   # 寬度佔整張圖的比例
        self.roi_height_ratio = 0.8  # 高度佔整張圖的比例

        # CLAHE 參數
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Sobel 參數
        self.ksize = 5              # Sobel kernel，或設為 -1 改用 Scharr
        self.mode = 'dy'            # 'dy' 或 'angle'
        self.tol_deg = 8.0          # 角度模式時的容許誤差（距±90°）

        # 形態學核
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # 快取用遮罩
        self._mask_shape = None
        self._mask = None

        self._printed_enc = False   # 僅首幀列印來源編碼

    # ---- 安全轉灰階 uint8 ----
    def _to_gray8(self, msg: Image) -> np.ndarray:
        enc = msg.encoding.lower() if msg.encoding else ""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if enc in ('mono8',):
            return img.astype(np.uint8)

        if enc in ('mono16', '16uc1', '16sc1') or (img.ndim == 2 and img.dtype in (np.uint16, np.int16)):
            return cv2.convertScaleAbs(img, alpha=255.0/65535.0, beta=0)

        if enc in ('bgr8', 'bgr16'):
            return cv2.cvtColor(cv2.convertScaleAbs(img), cv2.COLOR_BGR2GRAY)
        if enc in ('rgb8', 'rgb16'):
            return cv2.cvtColor(cv2.convertScaleAbs(img), cv2.COLOR_RGB2GRAY)
        if enc in ('bgra8', 'bgra16'):
            return cv2.cvtColor(cv2.convertScaleAbs(img), cv2.COLOR_BGRA2GRAY)
        if enc in ('rgba8', 'rgba16'):
            return cv2.cvtColor(cv2.convertScaleAbs(img), cv2.COLOR_RGBA2GRAY)

        if 'yuv422' in enc or 'uyvy' in enc or 'yuyv' in enc:
            bgr = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        if img.ndim == 3 and img.shape[2] >= 3:
            return cv2.cvtColor(cv2.convertScaleAbs(img), cv2.COLOR_BGR2GRAY)
        if img.ndim == 2:
            img = img.astype(np.float32)
            img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
            return img.astype(np.uint8)
        return cv2.convertScaleAbs(img)

    def _get_rect_mask(self, h: int, w: int) -> np.ndarray:
        """
        中心長方形遮罩 (uint8, 0/255)：
        寬度 = roi_width_ratio * w
        高度 = roi_height_ratio * h
        """
        if self._mask_shape != (h, w):
            mask = np.zeros((h, w), dtype=np.uint8)

            cx = w // 2
            cy = h // 2

            rect_w = int(w * self.roi_width_ratio)
            rect_h = int(h * self.roi_height_ratio)

            # 確保 rect_h > rect_w（高大於寬）
            if rect_h <= rect_w:
                rect_h = min(h, int(rect_w * 1.5))

            half_w = rect_w // 2
            half_h = rect_h // 2

            x1 = max(0, cx - half_w)
            x2 = min(w, cx + half_w)
            y1 = max(0, cy - half_h)
            y2 = min(h, cy + half_h)

            mask[y1:y2, x1:x2] = 255

            self._mask = mask
            self._mask_shape = (h, w)
        return self._mask

    def image_callback(self, msg: Image):
        try:
            if not self._printed_enc:
                self.get_logger().info(f"來源編碼: {msg.encoding}")
                self._printed_enc = True

            # 1) 轉灰階
            gray = self._to_gray8(msg)
            h, w = gray.shape

            # 2) CLAHE
            clahe_full = self.clahe.apply(gray)

            # 3) Sobel
            if self.ksize == -1:
                sobel_x = cv2.Scharr(clahe_full, cv2.CV_32F, 1, 0)
                sobel_y = cv2.Scharr(clahe_full, cv2.CV_32F, 0, 1)
            else:
                sobel_x = cv2.Sobel(clahe_full, cv2.CV_32F, 1, 0, ksize=self.ksize)
                sobel_y = cv2.Sobel(clahe_full, cv2.CV_32F, 0, 1, ksize=self.ksize)

            if self.mode == 'dy':
                sobel_resp = np.abs(sobel_y)
            else:
                angle = np.degrees(np.arctan2(sobel_y, sobel_x))
                dist_to_90 = np.minimum(np.abs(angle - 90.0), np.abs(angle + 90.0))
                angle_mask = (dist_to_90 <= self.tol_deg).astype(np.float32)
                mag = cv2.magnitude(sobel_x, sobel_y)
                sobel_resp = mag * angle_mask

            # 4) 只保留中心長方形區域
            mask = self._get_rect_mask(h, w).astype(np.float32) / 255.0
            sobel_resp *= mask

            # 5) 正規化到 0~255 並轉 uint8
            sobel_edges = cv2.normalize(sobel_resp, None, 0, 255, cv2.NORM_MINMAX)
            sobel_edges = sobel_edges.astype(np.uint8)

            # 6) 形態學：膨脹（若要 close 可改下面那行註解）
            dilated = cv2.dilate(sobel_edges, self.kernel, iterations=1)
            # dilated = cv2.morphologyEx(sobel_edges, cv2.MORPH_CLOSE, self.kernel, iterations=1)  # close 版

            # 顯示
            cv2.imshow("CLAHE Enhanced Image", clahe_full)
            cv2.imshow("Sobel Edge Detection", dilated)
            cv2.waitKey(1)

            # 發布 mono8
            out_msg = self.bridge.cv2_to_imgmsg(dilated, encoding="mono8")
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"❌ 處理影像失敗: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SobelEdgeDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
