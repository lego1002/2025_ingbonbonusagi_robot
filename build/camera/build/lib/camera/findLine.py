#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
import math
import glob

# ---- 固定參數（不寫入 JSON）----
ANG_TOL_DEG = 5.0  # 僅接受接近水平(0°/180°)的線段角度容許值

# ---- JSON 參數檔（鍵名保持不變）----
DEFAULT_PARAMS = {
    "roi_top_pct": 30,     # 從上往下的 ROI 起點 (%，0~90)
    "center_tol_px": 60,   # 線段中點與垂直中線的最大偏移 (px)
    "len_min": 80,         # 線段長度下限 (px)
    "len_max": 800,        # 線段長度上限 (px)
    "hough_thr": 50,       # Hough threshold
    "hough_min_len": 100,  # Hough minLineLength
    "hough_max_gap": 10,   # Hough maxLineGap
    "edge_thr": 0          # 保留欄位以維持相容性（本節點不使用）
}

def resolve_params_path() -> str:
    """
    盡量找到安裝後的 camera 套件內的 line_params.json：
    ~/.../robot_bartender/install/camera/lib/python*/site-packages/camera/line_params.json
    可用環境變數 RB_PARAMS_PATH 覆寫（完整檔案路徑）。
    找不到時，退回程式所在資料夾的 line_params.json。
    """
    # 1) 環境變數優先（若使用者手動指定）
    env_path = os.environ.get("RB_PARAMS_PATH", "").strip()
    if env_path:
        return env_path

    # 2) 在 Home 底下遞迴尋找 robot_bartender 安裝位置
    home = os.path.expanduser("~")
    pattern = os.path.join(
        home, "**", "robot_bartender", "install", "camera", "lib",
        "python*", "site-packages", "camera", "line_params.json"
    )
    matches = glob.glob(pattern, recursive=True)
    if matches:
        # 可依照最短/最新策略選擇，這裡選第一個即可
        return matches[0]

    # 3) 退回到此檔案所在資料夾
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "line_params.json")

PARAMS_FILE = resolve_params_path()

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
    # 防呆
    params["roi_top_pct"]   = max(0, min(90, int(params["roi_top_pct"])))
    params["center_tol_px"] = max(0, int(params["center_tol_px"]))
    params["len_min"]       = max(0, int(params["len_min"]))
    params["len_max"]       = max(params["len_min"], int(params["len_max"]))
    params["hough_thr"]     = max(1, int(params["hough_thr"]))
    params["hough_min_len"] = max(0, int(params["hough_min_len"]))
    params["hough_max_gap"] = max(0, int(params["hough_max_gap"]))
    params["edge_thr"]      = max(0, int(params["edge_thr"]))
    return params

def save_params(path: str, params: dict):
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(params, f, indent=2)
        print(f"[LineDetectionNode] Saved params to {path}")
    except Exception as e:
        print(f"[LineDetectionNode] Failed to save {path}: {e}")

def line_length(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

class LineDetectionNode(Node):
    def __init__(self):
        super().__init__('line_detection_node')

        self.bridge = CvBridge()
        self.params = load_params(PARAMS_FILE)

        # 訂閱 /camera/image_edge（mono8）
        self.sub = self.create_subscription(Image, '/camera/image_edge', self.image_callback, 10)
        # 發佈畫好線的影像
        self.pub = self.create_publisher(Image, '/camera/image_line', 10)

        self.get_logger().info(
            f"Line檢測節點已啟動（滑條即時調參，按 s 儲存到 {PARAMS_FILE}）"
        )

        # UI 視窗與滑條
        self.win_name = "Line Detection"
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win_name, 900, 700)

        # 滑條，用讀到/預設值初始化
        cv2.createTrackbar("roi_top_pct",   self.win_name, self.params["roi_top_pct"],   90,   lambda v: None)
        cv2.createTrackbar("center_tol_px", self.win_name, self.params["center_tol_px"], 2000, lambda v: None)
        cv2.createTrackbar("len_min",       self.win_name, self.params["len_min"],       2000, lambda v: None)
        cv2.createTrackbar("len_max",       self.win_name, self.params["len_max"],       5000, lambda v: None)
        cv2.createTrackbar("hough_thr",     self.win_name, self.params["hough_thr"],     500,  lambda v: None)
        cv2.createTrackbar("hough_min_len", self.win_name, self.params["hough_min_len"], 5000, lambda v: None)
        cv2.createTrackbar("hough_max_gap", self.win_name, self.params["hough_max_gap"], 1000, lambda v: None)
        cv2.createTrackbar("edge_thr",      self.win_name, self.params["edge_thr"],      255,  lambda v: None)  # 保留

        self._printed_enc = False

    def _read_trackbar_into_params(self):
        p = self.params
        p["roi_top_pct"]   = cv2.getTrackbarPos("roi_top_pct",   self.win_name)
        p["center_tol_px"] = cv2.getTrackbarPos("center_tol_px", self.win_name)
        p["len_min"]       = cv2.getTrackbarPos("len_min",       self.win_name)
        p["len_max"]       = cv2.getTrackbarPos("len_max",       self.win_name)
        p["hough_thr"]     = cv2.getTrackbarPos("hough_thr",     self.win_name)
        p["hough_min_len"] = cv2.getTrackbarPos("hough_min_len", self.win_name)
        p["hough_max_gap"] = cv2.getTrackbarPos("hough_max_gap", self.win_name)
        p["edge_thr"]      = cv2.getTrackbarPos("edge_thr",      self.win_name)

        # 防呆：len_max >= len_min
        if p["len_max"] < p["len_min"]:
            p["len_max"] = p["len_min"]
            cv2.setTrackbarPos("len_max", self.win_name, p["len_max"])

    def image_callback(self, msg: Image):
        try:
            if not self._printed_enc:
                self.get_logger().info(f"來源編碼: {msg.encoding}")
                self._printed_enc = True

            # 1) 取得邊緣圖（mono8）
            edge_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')  # HxW (uint8)
            H, W = edge_image.shape[:2]
            center_x = W // 2

            # 2) 讀滑條 → 更新參數
            self._read_trackbar_into_params()

            # 3) ROI
            roi_y0 = int(H * self.params["roi_top_pct"] / 100.0)
            roi_edge = edge_image[roi_y0:, :]

            # 4) Hough 直線偵測
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

                    # 長度過濾
                    L = line_length(x1, y1, x2, y2)
                    if not (self.params["len_min"] <= L <= self.params["len_max"]):
                        continue

                    # 僅接受接近水平的線（角度檢查）
                    ang = abs(math.degrees(math.atan2(y2 - y1, x2 - x1)))  # 0~180
                    if not (ang <= ANG_TOL_DEG or ang >= 180.0 - ANG_TOL_DEG):
                        continue

                    # 線段中點需靠近影像中線
                    xm = (x1 + x2) / 2.0
                    if abs(xm - center_x) > self.params["center_tol_px"]:
                        continue

                    cand.append((x1, y1, x2, y2))

                if cand:
                    # 由上到下排序（看 y 中點，注意是 ROI 座標）
                    cand.sort(key=lambda p: (p[1] + p[3]) / 2.0)
                    picked = cand[0]  # 第一條視為液面

            # 5) 可視化：把邊緣圖轉彩色後畫線
            out = cv2.cvtColor(edge_image, cv2.COLOR_GRAY2BGR)

            # 畫中線與 ROI 起點
            cv2.line(out, (center_x, 0), (center_x, H - 1), (150, 150, 150), 1)
            cv2.line(out, (0, roi_y0), (W - 1, roi_y0), (90, 0, 255), 1)

            if picked is not None:
                x1, y1, x2, y2 = picked
                cv2.line(out, (x1, y1 + roi_y0), (x2, y2 + roi_y0), (0, 255, 0), 3)  # 綠色水平線（液面）

            # 6) 顯示與存檔快捷鍵
            cv2.imshow(self.win_name, out)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                save_params(PARAMS_FILE, self.params)

            # 7) 發布結果
            annotated_msg = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
            annotated_msg.header = msg.header
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
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
