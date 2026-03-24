#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
import math

class RangeToScan(Node):
    def __init__(self):
        super().__init__('range2scan')
        
        # 订阅硬件发布的 /ultrasonic
        self.range_sub = self.create_subscription(
            Range, '/ultrasonic', self.range_callback, 10)
        
        # 发布转换后的 LaserScan 话题 /scan_ultrasonic
        self.scan_pub = self.create_publisher(LaserScan, '/scan_ultrasonic', 10)
        
        self.get_logger().info("超声波 Range → LaserScan 转换节点已启动")

    def range_callback(self, msg: Range):
        scan = LaserScan()
        
        # 固定配置（匹配超声波）
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_ultrasonic"  # 你的坐标系
        
        scan.angle_min = -0.15  # 左右15度
        scan.angle_max = 0.15
        scan.angle_increment = 0.01
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = msg.min_range
        scan.range_max = msg.max_range
        
        # 填充距离数据
        scan.ranges = [msg.range] * int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.intensities = [100.0] * len(scan.ranges)
        
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = RangeToScan()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
