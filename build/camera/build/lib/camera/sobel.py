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
        self.sub = self.create_subscription(Image, '/camera/image_rect', self.image_callback, 10)

        # 創建一個發布者，將處理後的影像發布到 /camera/image_edge
        self.pub = self.create_publisher(Image, '/camera/image_edge', 10)

        self.get_logger().info("Sobel邊緣檢測節點已啟動")
        
        # 視窗大小
        cv2.namedWindow("Sobel Edge Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Sobel Edge Detection", 800, 600)
        
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

            # Sobel 邊緣檢測
            sobel_x = cv2.Sobel(clahe_image, cv2.CV_64F, 1, 0, ksize=3)  # 水平方向梯度
            sobel_y = cv2.Sobel(clahe_image, cv2.CV_64F, 0, 1, ksize=3)  # 垂直方向梯度

            # 合併 X 和 Y 方向的梯度
            sobel_edges = cv2.magnitude(sobel_x, sobel_y)

            # 轉換回 uint8 格式
            sobel_edges = np.uint8(np.clip(sobel_edges, 0, 255))

            # 膨脹操作 - 使用 3x3 的核
            kernel = np.ones((3, 3), np.uint8)  # 定義膨脹操作的核
            dilated_edges = cv2.dilate(sobel_edges, kernel, iterations=1)

            # 顯示 CLAHE 增強後的影像
            cv2.imshow("CLAHE Enhanced Image", clahe_image)

            # 顯示 Sobel 邊緣檢測結果
            cv2.imshow("Sobel Edge Detection", dilated_edges)  # 顯示膨脹後的邊緣圖像
            cv2.waitKey(1)

            # 把處理後的 Sobel 邊緣影像發布到 /camera/image_edge
            image_edge_msg = self.bridge.cv2_to_imgmsg(dilated_edges, encoding="mono8")
            self.pub.publish(image_edge_msg)

        except Exception as e:
            self.get_logger().error(f"❌ 處理影像失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SobelEdgeDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()  # 關閉所有 OpenCV 視窗
        rclpy.shutdown()

if __name__ == '__main__':
    main()
