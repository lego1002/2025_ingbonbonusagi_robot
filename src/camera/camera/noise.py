import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageFilterNode(Node):
    def __init__(self):
        super().__init__('image_filter_node')
        
        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'camera/image_filtered', 10)
        
        self.get_logger().info("Image Filter Node 已啟動")
        
        cv2.namedWindow("Filtered Image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Filtered Image", 800, 600)

    def image_callback(self, msg):
        try:
            # 將 ROS 訊息轉換為 OpenCV 影像
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 分割影像為 R、G、B 通道
            (B, G, R) = cv2.split(frame)

            # 分別對 R、G、B 通道進行頻域處理
            def remove_banding_from_channel(channel):
                f = np.fft.fft2(channel)  # 進行傅立葉變換
                fshift = np.fft.fftshift(f)

                # 高通濾波：只保留頻譜中心的高頻部分
                rows, cols = channel.shape
                crow, ccol = rows // 2, cols // 2
                mask = np.zeros(fshift.shape, np.uint8)  # 初始化為零的 mask

                r = 30  # 保留頻譜圖中心部分的半徑
                mask[crow-r:crow+r, ccol-r:ccol+r] = 1  # 只保留中心區域

                # 應用濾波器
                fshift = fshift * mask

                # 反傅立葉變換
                f_ishift = np.fft.ifftshift(fshift)
                img_back = np.fft.ifft2(f_ishift)
                img_back = np.abs(img_back)

                # 正規化到 [0, 255]
                img_back = np.uint8(np.clip(img_back, 0, 255))
                return img_back

            # 分別處理 R、G、B 通道
            R_processed = remove_banding_from_channel(R)
            G_processed = remove_banding_from_channel(G)
            B_processed = remove_banding_from_channel(B)

            # 合併處理後的通道
            processed_frame = cv2.merge([B_processed, G_processed, R_processed])

            # 顯示處理後的影像
            cv2.imshow("Filtered Image", processed_frame)
            cv2.waitKey(1)  # 等待1毫秒顯示

            # 發布處理後的影像
            img_msg = self.bridge.cv2_to_imgmsg(processed_frame, encoding="bgr8")
            self.publisher_.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"處理影像時發生錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("關閉 Image Filter Node")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()  # 關閉所有顯示窗口
        rclpy.shutdown()

if __name__ == '__main__':
    main()
