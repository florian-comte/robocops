#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from numpy import linspace, inf
from math import sin
from sensor_msgs.msg import LaserScan
class ScanFilter(Node):
    def __init__(self):
        super().__init__('stretch_scan_filter')
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_filter_callback, 10)

        self.width = 1
        self.extent = self.width / 2.0
    def scan_filter_callback(self,msg):
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = [r * sin(theta) if (theta < -0.78 or theta > 0.78) else inf for r,theta in zip(msg.ranges, angles)]
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]
        msg.ranges = new_ranges
        self.pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    scan_filter = ScanFilter()
    rclpy.spin(scan_filter)
    scan_filter.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()