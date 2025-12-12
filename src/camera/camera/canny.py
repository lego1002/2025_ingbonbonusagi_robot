import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CannyEdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('canny_edge_detection_node')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.get_logger().info("Canny邊緣檢測節點已啟動")
        
        # 視窗大小
        cv2.namedWindow("Canny Edge Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Canny Edge Detection", 800, 600)
        
        cv2.namedWindow("CLAHE Enhanced Image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("CLAHE Enhanced Image", 800, 600)

    def image_callback(self, msg):
        try:
            # 將 ROS 影像訊息轉換為 OpenCV 格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 轉換為灰階影像
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 創建 CLAHE 實例，設置最大對比度增強限制與區塊大小
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

            # 應用 CLAHE 來增強對比度
            clahe_image = clahe.apply(gray)

            # 使用 Canny 邊緣檢測
            edges = cv2.Canny(clahe_image, 30, 200)

            # 顯示 CLAHE 增強後的影像
            cv2.imshow("CLAHE Enhanced Image", clahe_image)

            # 顯示 Canny 邊緣檢測結果
            cv2.imshow("Canny Edge Detection", edges)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ 處理影像失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgeDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()  # 關閉所有 OpenCV 視窗
        rclpy.shutdown()

if __name__ == '__main__':
    main()




