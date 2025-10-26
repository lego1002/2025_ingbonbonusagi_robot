import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
import math

PARAMS_FILE = "line_params.json"

# 預設參數（與調參工具一致的鍵）
DEFAULT_PARAMS = {
    "roi_top_pct": 30,     # 從上往下的 ROI 起點 (%，0~90)
    "center_tol_px": 60,   # 線段中點與垂直中線的最大偏移 (px)
    "len_min": 80,         # 線段長度下限 (px)
    "len_max": 800,        # 線段長度上限 (px)
    "hough_thr": 50,       # Hough threshold
    "hough_min_len": 100,  # Hough minLineLength
    "hough_max_gap": 10,   # Hough maxLineGap
    "edge_thr": 0          # Sobel 圖二值化門檻 (0=關閉，直接用原圖)
}

def load_params(path: str):
    params = DEFAULT_PARAMS.copy()
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                saved = json.load(f)
            for k in params:
                if k in saved:
                    params[k] = saved[k]
            print(f"[LineDetectionNode] Loaded params from {path}")
        except Exception as e:
            print(f"[LineDetectionNode] Failed to load {path}: {e}. Using defaults.")
    else:
        print(f"[LineDetectionNode] {path} not found. Using defaults.")
    # 基本防呆
    params["roi_top_pct"] = max(0, min(90, int(params["roi_top_pct"])))
    params["center_tol_px"] = max(0, int(params["center_tol_px"]))
    params["len_min"] = max(0, int(params["len_min"]))
    params["len_max"] = max(params["len_min"], int(params["len_max"]))
    params["hough_thr"] = max(1, int(params["hough_thr"]))
    params["hough_min_len"] = max(0, int(params["hough_min_len"]))
    params["hough_max_gap"] = max(0, int(params["hough_max_gap"]))
    params["edge_thr"] = max(0, int(params["edge_thr"]))
    return params

def line_length(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

class LineDetectionNode(Node):
    def __init__(self):
        super().__init__('line_detection_node')

        self.bridge = CvBridge()
        self.params = load_params(PARAMS_FILE)

        # 訂閱 Sobel 邊緣影像（mono8）
        self.sub = self.create_subscription(Image, '/camera/image_edge', self.image_callback, 10)
        # 發佈畫好線的影像
        self.pub = self.create_publisher(Image, '/camera/image_line', 10)

        self.get_logger().info("Line檢測節點已啟動（使用 line_params.json 的參數）")

    def image_callback(self, msg: Image):
        try:
            # 取得灰階邊緣圖
            edge_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')  # HxW (uint8)
            H, W = edge_image.shape[:2]
            center_x = W // 2

            # 取 ROI
            roi_y0 = int(H * self.params["roi_top_pct"] / 100.0)
            roi = edge_image[roi_y0:, :]

            # 可選二值化（edge_thr > 0 時啟用）
            if self.params["edge_thr"] > 0:
                _, roi_edge = cv2.threshold(roi, self.params["edge_thr"], 255, cv2.THRESH_BINARY)
            else:
                roi_edge = roi

            # Hough 直線偵測
            linesP = cv2.HoughLinesP(
                roi_edge,
                rho=1,
                theta=np.pi / 180,
                threshold=self.params["hough_thr"],
                minLineLength=self.params["hough_min_len"],
                maxLineGap=self.params["hough_max_gap"]
            )

            picked = None  # (x1,y1,x2,y2) in ROI coords
            if linesP is not None and len(linesP) > 0:
                cand = []
                for l in linesP:
                    x1, y1, x2, y2 = l[0]

                    # 長度過濾（你自定義的外層長度條件）
                    L = line_length(x1, y1, x2, y2)
                    if not (self.params["len_min"] <= L <= self.params["len_max"]):
                        continue

                    # 線段中點需靠近影像中線
                    xm = (x1 + x2) / 2.0
                    if abs(xm - center_x) > self.params["center_tol_px"]:
                        continue

                    cand.append((x1, y1, x2, y2))

                if cand:
                    # 由上到下排序（看 y 中點，注意是 ROI 座標）
                    cand.sort(key=lambda p: (p[1] + p[3]) / 2.0)
                    picked = cand[0]  # 第一條就是液面

            # 準備輸出影像（彩色）
            out = cv2.cvtColor(edge_image, cv2.COLOR_GRAY2BGR)

            # 可視化：畫出中線與 ROI 起點（如不想顯示，可註解）
            cv2.line(out, (center_x, 0), (center_x, H - 1), (150, 150, 150), 1)
            cv2.line(out, (0, roi_y0), (W - 1, roi_y0), (90, 0, 255), 1)

            if picked is not None:
                x1, y1, x2, y2 = picked
                # 轉回全圖座標（加上 ROI 偏移）
                cv2.line(out, (x1, y1 + roi_y0), (x2, y2 + roi_y0), (0, 255, 0), 3)  # 液面（綠）

            # 發布
            annotated_msg = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
            annotated_msg.header = msg.header  # 保留時間戳與 frame_id
            self.pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f"❌ 處理影像失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
