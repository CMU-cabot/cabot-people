import signal
import sys

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

class ScanReceiver(Node):

    def __init__(self):
        super().__init__('scan_receiver')
        self.scan_sub = self.create_subscription(
                PointCloud2, 
                '/velodyne_points',
                self.scan_cb,
                10)
        return

    def scan_cb(self, msg):
        header = msg.header
        height = msg.height
        width = msg.width
        pt_field = msg.fields
        pt_step = msg.point_step
        rw_step = msg.row_step
        pt_data = msg.data
        print("======================Point cloud received!========================")
        print(header)
        print(height)
        print(width)
        print(pt_field)
        print(pt_step)
        print(rw_step)
        return

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
