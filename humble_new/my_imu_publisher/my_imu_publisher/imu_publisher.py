#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.threshold = 0.85  # meters
        self.angle_window = math.radians(40)  # ±15 degrees = 30° window

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_readings = len(msg.ranges)

    # Define asymmetric window
        left_angle = math.radians(60.0)   # +60° left
        right_angle = math.radians(10.0)  # -10° right

        center_index = int((0.0 - angle_min) / angle_increment)
        left_index = int(left_angle / angle_increment)
        right_index = int(right_angle / angle_increment)

        start_index = max(center_index - right_index, 0)
        end_index = min(center_index + left_index, num_readings - 1)

    # Extract the ranges inside the asymmetric window
        front_ranges = msg.ranges[start_index:end_index + 1]

    # Filter out invalid ranges (0.0 and inf)
        valid_ranges = [r for r in front_ranges if 0.01 < r < float('inf')]

    # Decide obstacle presence
        near_obstacle = any(r < self.threshold for r in valid_ranges)
        self.obstacle_pub.publish(Bool(data=near_obstacle))


def main():
    rclpy.init()
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
