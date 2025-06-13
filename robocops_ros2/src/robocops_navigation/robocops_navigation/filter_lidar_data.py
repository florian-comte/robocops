#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import time
from numpy import linspace, inf
from math import radians, degrees
from sensor_msgs.msg import LaserScan
class ScanFilter(Node):
    def __init__(self):
        super().__init__('front_scan_filter')
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 50)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_filter_callback, 50)

        self.front_angle_limit = radians(200 / 2)

    def scan_filter_callback(self, msg):
        num_points = len(msg.ranges)
        angles = linspace(msg.angle_min, msg.angle_max, num_points)

        # self.get_logger().info(f"Scan received with {num_points} points.")
        # self.get_logger().info(f"Angle range: {degrees(msg.angle_min):.2f}° to {degrees(msg.angle_max):.2f}°")
        
        # Filter the scan
        new_ranges = []
        kept_count = 0
        for r, theta in zip(msg.ranges, angles):
            if -self.front_angle_limit + radians(180 / 2) <= theta <= self.front_angle_limit + radians(180 / 2):
                new_ranges.append(r)
                kept_count += 1
            else:
                new_ranges.append(inf)

        # self.get_logger().info(f"Filtered scan: kept {kept_count} out of {num_points} points ({100 * kept_count / num_points:.1f}%)")
        
        msg.ranges = new_ranges
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    scan_filter = ScanFilter()
    rclpy.spin(scan_filter)
    scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
