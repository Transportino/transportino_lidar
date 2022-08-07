from copy import deepcopy
import rclpy
from rclpy.node import Node

from typing import List
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from transportino_lidar.mb_1r2t import LidarModule, ScanPoint

import time

from math import pi

class LidarNode(Node):

    default_scan_msg : LaserScan
    lidar_module : LidarModule
    
    def __init__(self):
        super().__init__(node_name='lidar_node', namespace='transportino')
        self.get_logger().info('Connecting to lidar module..')
        self.lidar_module = LidarModule('/dev/ttyUSB0', self.on_scan)
        
        default_header = Header()
        default_header.frame_id = 'laser'

        self.default_scan_msg = LaserScan()
        self.default_scan_msg.header = default_header
        self.default_scan_msg.angle_min = 0.0
        self.default_scan_msg.angle_max = 2 * pi
        self.default_scan_msg.range_min = 0.11
        self.default_scan_msg.range_max = 8.0

        self.publisher_ = self.create_publisher(LaserScan, 'lidar', 10)
        self.get_logger().info('Lidar node started.')

    def update(self):
        self.lidar_module.update_data()

    def on_scan(self, scan_points : List[ScanPoint], scan_time: float) :
        scan_msg : LaserScan = deepcopy(self.default_scan_msg)
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.angle_increment = (2 * pi) / len(scan_points)
        scan_msg.scan_time = scan_time
        scan_msg.time_increment = scan_time / len(scan_points)

        for scan_point in scan_points:
            scan_msg.ranges.append(scan_point.range)
            scan_msg.intensities.append(scan_point.intensity)

        self.publisher_.publish(scan_msg)


def main():
    rclpy.init()

    lidar_node = LidarNode()

    while rclpy.ok():
        try:
            lidar_node.update()
        except KeyboardInterrupt:
            break

    lidar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
