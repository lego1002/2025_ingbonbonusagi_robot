#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, Tuple

class NoiseFilterNode(Node):
    def __init__(self):
        super().__init__('noise_filter_node')

        # --- 參數 ---
        self.peaks_norm = np.array([[0.0,  0.0513393],   # (u0, v0)
                                    [0.0, -0.0513393]],  # (u0,-v0)
                                   dtype=np.float32)
        self.r_pix = 2  # notch 半徑（像素）

        # --- ROS IO ---
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image,
                                            '/camera/image_rect',
                                            self.image_callback,
                                            10)
        self.pub = self.create_publisher(Image,
                                         '/camera/noise_filtered',
                                         10)

        # --- 影像顯示視窗 ---
        cv2.namedWindow('noise_filtered (gray)', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('noise_filtered (gray)', 800, 600)

        self.get_logger().info('NoiseFilterNode 已啟動: /camera/image_rect -> /camera/noise_filtered')

        # 快取 notch mask（影像尺寸變化時會重建）
        self._mask_cache_shape: Optional[Tuple[int,int]] = None
        self._H_shifted: Optional[np.ndarray] = None  # 中心化(shifted)遮罩

    # =============== 工具：建立中心化(fftshift後)的 notch 遮罩 ===============
    def build_notch_mask_shifted(self, M: int, N: int) -> np.ndarray:
        """
        回傳中心化(fftshift後)的 2D notch 遮罩 H ∈ {0,1}, shape=(M,N)
        u 對應寬 N（x/欄），v 對應高 M（y/列），peaks_norm ∈ [-0.5, 0.5]
        """
        # 頻率索引座標（中心在 0,0）
        kx, ky = np.meshgrid(np.arange(-N//2, N//2, dtype=np.int32),
                             np.arange(-M//2, M//2, dtype=np.int32))

        H = np.ones((M, N), dtype=np.float32)

        for (u0, v0) in self.peaks_norm:
            # 歸一化頻率 → 像素索引（以中心為 0）
            cx = int(np.round(u0 * N))
            cy = int(np.round(v0 * M))

            # 共軛對稱（四個）中心點
            centers = [( cx,  cy),
                       (-cx, -cy),
                       ( cx, -cy),
                       (-cx,  cy)]

            for (mx, my) in centers:
                # 半徑內設為 0
                mask = (kx - mx) ** 2 + (ky - my) ** 2 <= (self.r_pix ** 2)
                H[mask] = 0.0

        return H

    # =========================== 主要回呼 ===========================
    def image_callback(self, msg: Image):
        try:
            # 1) 影像 → 灰階 float32
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_f = gray.astype(np.float32) / 255.0

            M, N = gray_f.shape

            # 2) 建或取用 notch 遮罩（中心化版）
            if self._mask_cache_shape != (M, N) or self._H_shifted is None:
                self._H_shifted = self.build_notch_mask_shifted(M, N)
                self._mask_cache_shape = (M, N)
                self.get_logger().info(f'重建 notch mask, size=({M},{N}), r={self.r_pix}')

            # 3) FFT（numpy，並 fftshift 讓低頻在中心）
            F = np.fft.fft2(gray_f)
            F_shift = np.fft.fftshift(F)

            # 4) 濾波：乘上中心化遮罩
            Ff_shift = F_shift * self._H_shifted

            # 5) 反中心化 + IFFT -> 實部
            Ff = np.fft.ifftshift(Ff_shift)
            gray_filtered = np.real(np.fft.ifft2(Ff))
            # 夾到 [0,1]
            gray_filtered = np.clip(gray_filtered, 0.0, 1.0)

            # 6) 顯示
            show = (gray_filtered * 255.0).astype(np.uint8)
            cv2.imshow('noise_filtered (gray)', show)
            # waitKey 必須呼叫以刷新 UI；設很小值以不阻塞
            cv2.waitKey(1)

            # 7) 發佈 mono8
            out_msg = self.bridge.cv2_to_imgmsg(show, encoding='mono8')
            out_msg.header = msg.header  # 保留時間戳與 frame_id
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'image_callback 錯誤: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = NoiseFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
