#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import subprocess
import shutil
import os
import sys
import contextlib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


@contextlib.contextmanager
def suppress_stderr():
    """
    暫時把整個行程的 stderr 重導到 /dev/null。
    用來吃掉 libjpeg 的 'Corrupt JPEG data ...' 訊息。
    注意：這段期間其他 C 函式寫到 stderr 也會被吃掉。
    """
    try:
        fd = sys.stderr.fileno()
    except Exception:
        # 沒有標準錯誤（很少見），直接不做事
        yield
        return

    saved = os.dup(fd)
    try:
        with open(os.devnull, 'w') as devnull:
            os.dup2(devnull.fileno(), fd)
            yield
    finally:
        os.dup2(saved, fd)
        os.close(saved)


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # -------- ROS2 參數 --------
        self.declare_parameter('device', '/dev/video2')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('pixel_format', 'MJPG')  # 固定用 MJPG
        self.declare_parameter('exposure_mode_manual', True)
        self.declare_parameter('exposure_time_abs', 156)
        self.declare_parameter('dynamic_fps', False)
        self.declare_parameter('awb_auto', False)
        self.declare_parameter('af_auto', False)
        self.declare_parameter('topic', 'camera/image_raw')
        self.declare_parameter('buffersize', 1)
        self.declare_parameter('show_window', False)
        self.declare_parameter('window_name', 'Camera V4L2 (Auto Config)')

        p = lambda k: self.get_parameter(k).get_parameter_value()
        self.device = p('device').string_value or '/dev/video2'
        self.W = int(p('width').integer_value or 1920)
        self.H = int(p('height').integer_value or 1080)
        self.FPS_REQ = int(p('fps').integer_value or 30)
        self.pixel_format = (p('pixel_format').string_value or 'MJPG').upper()
        self.expo_manual = bool(p('exposure_mode_manual').bool_value)
        self.expo_abs = int(p('exposure_time_abs').integer_value or 156)
        self.dynamic_fps = bool(p('dynamic_fps').bool_value)
        self.awb_auto = bool(p('awb_auto').bool_value)
        self.af_auto  = bool(p('af_auto').bool_value)
        self.topic = p('topic').string_value or 'camera/image_raw'
        self.buffersize = max(1, int(p('buffersize').integer_value or 1))
        self.show_window = bool(p('show_window').bool_value)
        self.window_name = p('window_name').string_value or 'Camera V4L2 (Auto Config)'

        self.publisher_ = self.create_publisher(Image, self.topic, 10)
        self.bridge = CvBridge()

        # -------- v4l2-ctl 設定硬體 --------
        self.setup_v4l2()

        # -------- 開相機 --------
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("無法開啟攝影機！")
            raise RuntimeError("open camera failed")

        # FOURCC → 寬高 → FPS → buffer
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.cap.set(cv2.CAP_PROP_FPS,          self.FPS_REQ)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffersize)
        except Exception:
            pass

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps_rep = self.cap.get(cv2.CAP_PROP_FPS)
        fcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fmt = "".join([chr((fcc >> (8*i)) & 0xFF) for i in range(4)])
        self.get_logger().info(
            f"OpenCV 回報: {w}x{h} @ {fps_rep:.2f}fps, FOURCC={fmt}, buffer={self.buffersize}"
        )

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 800, 600)

        # -------- 專用抓取執行緒（只維護最新幀）--------
        self._latest_frame = None
        self._lock = threading.Lock()
        self._grab_running = True
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        # 發布計時器（不阻塞抓取）
        self.timer = self.create_timer(1.0/120.0, self.publish_frame)

        # FPS 統計（發布端）
        self._last = time.time()
        self._cnt = 0
        self.get_logger().info(f'📷 Camera Node 已啟動，發布到 {self.topic}')

    # ========== 抓取執行緒 ==========
    def _grab_loop(self):
        """
        read() 直接取幀，僅保留最新幀。
        讀取時暫時關掉 stderr，吃掉 libjpeg 的 Corrupt JPEG data 訊息。
        """
        while self._grab_running:
            with suppress_stderr():
                ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.001)
                continue
            with self._lock:
                self._latest_frame = frame

    # ========== v4l2 設定 ==========
    def run_cmd(self, cmd):
        try:
            self.get_logger().info("$ " + " ".join(cmd))
            r = subprocess.run(cmd, capture_output=True, text=True)
            if r.stdout.strip():
                self.get_logger().info(r.stdout.strip())
            if r.stderr.strip():
                self.get_logger().info(r.stderr.strip())
        except FileNotFoundError:
            self.get_logger().warn("找不到 v4l2-ctl，請先安裝： sudo apt install v4l-utils")
        except Exception as e:
            self.get_logger().warn(f"指令失敗: {cmd} → {e}")

    def setup_v4l2(self):
        if shutil.which("v4l2-ctl") is None:
            self.get_logger().warn("未安裝 v4l2-ctl，跳過硬體層設定")
            return
        # 1) 像素格式/解析度（MJPG）
        self.run_cmd([
            "v4l2-ctl", "-d", self.device, "--set-fmt-video",
            f"width={self.W},height={self.H},pixelformat=MJPG"
        ])
        # 2) FPS
        self.run_cmd(["v4l2-ctl", "-d", self.device, "--set-parm", str(self.FPS_REQ)])
        # 3) 曝光（手動 or 自動）
        if self.expo_manual:
            self.run_cmd(["v4l2-ctl", "-d", self.device, "--set-ctrl", "auto_exposure=1"])
            self.run_cmd([
                "v4l2-ctl", "-d", self.device, "--set-ctrl",
                f"exposure_time_absolute={int(self.expo_abs)}"
            ])
        else:
            self.run_cmd(["v4l2-ctl", "-d", self.device, "--set-ctrl", "auto_exposure=3"])
        # 4) 關動態降幀、WB/AF
        self.run_cmd([
            "v4l2-ctl", "-d", self.device, "--set-ctrl",
            f"exposure_dynamic_framerate={1 if self.dynamic_fps else 0}"
        ])
        self.run_cmd([
            "v4l2-ctl", "-d", self.device, "--set-ctrl",
            f"white_balance_automatic={1 if self.awb_auto else 0}"
        ])
        self.run_cmd([
            "v4l2-ctl", "-d", self.device, "--set-ctrl",
            f"focus_automatic_continuous={1 if self.af_auto else 0}"
        ])
        # 檢視
        self.run_cmd([
            "v4l2-ctl", "-d", self.device, "--get-fmt-video",
            "--get-parm", "--list-ctrls"
        ])
        self.get_logger().info("✅ V4L2 參數設定完成")

    # ========== 發布 ==========
    def publish_frame(self):
        with self._lock:
            frame = self._latest_frame
        if frame is None:
            return

        self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

        if self.show_window:
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

        self._cnt += 1
        now = time.time()
        if now - self._last >= 1.0:
            self.get_logger().info(f"輸出 FPS ≈ {self._cnt}")
            self._cnt = 0
            self._last = now

    def destroy_node(self):
        try:
            self._grab_running = False
            if hasattr(self, '_grab_thread'):
                self._grab_thread.join(timeout=1.0)
            if hasattr(self, 'cap') and self.cap:
                self.cap.release()
            if self.show_window:
                cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("關閉 Camera Node")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
