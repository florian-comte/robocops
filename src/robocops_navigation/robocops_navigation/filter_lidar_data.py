#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from numpy import linspace, inf
from math import radians
from sensor_msgs.msg import LaserScan

class ScanFilter(Node):
    def __init__(self):
        super().__init__('front_scan_filter')
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_filter_callback, 10)

        self.front_angle_limit = radians(220 / 2) 

    def scan_filter_callback(self, msg):
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        new_ranges = [
            r if -self.front_angle_limit <= theta <= self.front_angle_limit else inf
            for r, theta in zip(msg.ranges, angles)
        ]

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
