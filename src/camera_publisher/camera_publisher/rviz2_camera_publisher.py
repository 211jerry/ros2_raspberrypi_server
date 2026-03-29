import rclpy
from rclpy.node import Node
import cv2
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile

# 树莓派Ubuntu Server CSI专用
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # QoS配置（兼容RViz）
        qos = QoSProfile(depth=1)
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', qos)
        self.bridge = CvBridge()

        # 打开CSI摄像头（V4L2原生驱动）
        self.get_logger().info('正在连接CSI摄像头...')
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        # CSI摄像头初始化等待（关键）
        time.sleep(2)

        if not self.cap.isOpened():
            self.get_logger().fatal('❌ 摄像头打开失败')
            rclpy.shutdown()
            return

        # 最低分辨率，保证稳定
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.get_logger().info('✅ CSI摄像头已启动')

        # 10FPS发布
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # 发布图像
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
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
