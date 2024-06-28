import signal
import sys

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

import open3d as o3d

class ScanReceiver(Node):

    def __init__(self):
        super().__init__('scan_receiver')
        self.scan_sub = self.create_subscription(
                PointCloud2, 
                '/velodyne_points',
                self.scan_cb,
                10)

        self.pointcloud = None

        self.ring_limit = self.declare_parameter('ring_limit', -1).value
        self.scan_max_range = self.declare_parameter('scan_max_range', 100).value
        return

    def scan_cb(self, msg):
        # Stores pointcloud when the node receives one.

        # Pointcloud format
        # (x, y, z, intensity, ring number [e.g. 0-15])
        parsed_pointcloud = point_cloud2.read_points(msg, skip_nans=True)
        num_raw_pt = len(parsed_pointcloud)
        if num_raw_pt > 0:
            self.pointcloud = np.zeros((num_raw_pt, len(parsed_pointcloud[0])))
            validation_array = np.zeros(num_raw_pt)
            for i, elem in enumerate(parsed_pointcloud):
                self.pointcloud[i, :] = list(elem)
                if self._is_valid_pt(elem):
                    validation_array[i] = 1
            self.pointcloud = self.pointcloud[validation_array == 1]
        else:
            self.pointcloud = None
        print(self.pointcloud)
        return

    def _is_valid_pt(self, elem):
        # test if ring specified
        if not (self.ring_limit == -1):
            ring = elem[4]
            if not ring == self.ring_limit:
                return False

        # test if point distance passes max range
        if self.scan_max_range < 100:
            x = elem[0]
            y = elem[1]
            dist = np.sqrt(x*x + y*y)
            if dist > self.scan_max_range:
                return False

        return True


def main():
    rclpy.init()
    scan_receiver = ScanReceiver()

    try:
        rclpy.spin(scan_receiver)
    except:
        scan_receiver.get_logger().info("Shutting down")

    scan_receiver.destroy_node()
    rclpy.shutdown()
    return


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)
    return

signal.signal(signal.SIGINT, receiveSignal)

if __name__ == '__main__':
    main()
