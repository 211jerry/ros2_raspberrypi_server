import rclpy
from rclpy.node import Node
import cv2
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        qos = QoSProfile(depth=1)

        # 核心：仅发布压缩图像，唯一类型，匹配前端
        self.publisher_ = self.create_publisher(
            CompressedImage, 
            '/camera/image_raw', 
            qos
        )
        
        self.bridge = CvBridge()

        # 树莓派CSI摄像头
        self.get_logger().info('正在启动CSI摄像头...')
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        time.sleep(2)
        
        if not self.cap.isOpened():
            self.get_logger().fatal('❌ 摄像头打开失败')
            rclpy.shutdown()
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.get_logger().info('✅ 压缩图像发布中 (适配前端+RViz2)')

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # 发布JPG压缩图像
        msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpg")
        msg.header.frame_id = "camera_link"
        self.publisher_.publish(msg)

    def __del__(self):
        if self.cap:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()