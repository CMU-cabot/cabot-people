import signal
import sys
import queue
import threading
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from cabot_msgs.msg import PoseLog

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from cabot_msgs.srv import LookupTransform

import open3d as o3d

class ScanReceiver(Node):

    def __init__(self):
        super().__init__('scan_receiver')

        state_update_callback_group = MutuallyExclusiveCallbackGroup()
        transform_lookup_callback_group = MutuallyExclusiveCallbackGroup()
        self.scan_sub = self.create_subscription(
            PointCloud2, 
            '/velodyne_points_cropped',
            self.scan_cb,
            10,
            callback_group = state_update_callback_group
        )
        self.scan_sub = self.create_subscription(
            PoseLog, 
            '/cabot/pose_log',
            self.pose_cb,
            10,
            callback_group = state_update_callback_group
        )
        
        self.pcl_debug_pub = self.create_publisher(
            PointCloud2,
            "/map_velodyne_points",
            10,
            callback_group = transform_lookup_callback_group
        )

        self.pointcloud = np.array([])
        self.pointcloud_prev = np.array([])
        self.pointcloud_complete = np.array([])

        self.pose = None
        self.prev_pose = None
        self.curr_pose = None
        self.curr_time = 0
        self.prev_time = 0

        self._ring_limit = self.declare_parameter('ring_limit', -1).value
        self._scan_max_range = self.declare_parameter('scan_max_range', 100).value
        self._history_window = self.declare_parameter('history_window', 8).value
        self._future_window = self.declare_parameter('future_window', 8).value
        self._low_level_pos_threshold = self.declare_parameter('low_level_pos_threshold', 0.25).value
        self._high_level_pos_threshold = self.declare_parameter('high_level_pos_threshold', 1).value
        self._high_level_vel_threshold = self.declare_parameter('high_level_vel_threshold', 1).value
        self._high_level_ori_threshold = self.declare_parameter('high_level_ori_threshold', 1).value

        self.lookup_transform_service = self.create_client(LookupTransform, '/lookup_transform')

        self.pointcloud_history = queue.Queue(self._history_window)
        self.pointcloud_history_np = []  # list of numpys
        self.is_history_complete = False

        return
    
    def lookup_transform(self, source, target):
        # Look up and return tf transformation from source frame to target frame

        self.get_logger().info(f"lookup_transform({source}, {target})")

        req = LookupTransform.Request()
        req.target_frame = target
        req.source_frame = source
        if not self.lookup_transform_service.wait_for_service(timeout_sec=1.0):
            raise Exception("lookup transform service is not available")
        
        # Borrowed from BufferProxy in cabot-navigation/cabot_ui/cabot_ui/navigation.py
        future = self.lookup_transform_service.call_async(req)
        print("stuck here")
        rclpy.spin_until_future_complete(self, future)
        print("not stuck here")

        """
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()
        future.add_done_callback(unblock)
        # Check future.done() before waiting on the event.
        # The callback might have been added after the future is completed,
        # resulting in the event never being set.
        if not future.done():
            if not event.wait(10.0):
                # Timed out. remove_pending_request() to free resources
                self.lookup_transform_service.remove_pending_request(future)
                raise Exception("timeout")
        if future.exception() is not None:
            raise future.exception()
        """

        # sync call end here
        result = future.result()
        if result.error.error > 0:
            raise Exception(result.error.error_string)
        return result.transform

    def scan_cb(self, msg):
        # Stores pointcloud when the node receives one.

        if self.pose is None:
            return

        self.curr_pose = self.pose
        self.curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = self.curr_time - self.prev_time

        self.get_logger().info("pcl received")

        # Pointcloud format
        # (x, y, z, intensity, ring number [e.g. 0-15])
        # Pointcloud complete format
        # (x, y, z, intensity, ring, group id, group center x, group center y, vel_x, vel_y)
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
            self.pointcloud = np.array([])

        # transform pointcloud to map frame
        self.get_logger().info("lidar transform start")
        lidar_to_map = self.lookup_transform('velodyne', 'map')
        self.get_logger().info("transform received")
        transformed_cloud = self._do_transform(self.pointcloud, lidar_to_map, msg.header)
        self.get_logger().info("lidar transform end")

        print("--------pcl debug--------")
        print(self.pointcloud[0])
        print(transformed_cloud[0])

        self.pointcloud_complete = self._calc_pointcloud_group_vel(
                                            self.pointcloud_prev, 
                                            self.pointcloud, 
                                            self.prev_pose, 
                                            self.curr_pose,
                                            dt)
        self.pointcloud_prev = self.pointcloud_complete 
        self.prev_pose = self.curr_pose
        self.prev_time = self.curr_time

        if self.pointcloud_history.qsize() == self._history_window :
            self.is_history_complete = True
            # pop the oldest point cloud from the queue
            tmp = self.pointcloud_history.get(block=False)
        self.pointcloud_history.put(self.pointcloud, block=False)
        
        return
    
    def pose_cb(self, msg):
        self.pose = msg.pose
        return
    
    def _do_transform(self, pointcloud, transform, header=None):
        # This function transforms the pointcloud in msg to the transformation defined in
        # transform. Transform is obtained by calling lookup_transform.
        # If header is provided, transformed pointclouds will be published

        pointcloud_coord = pointcloud[:, :3]

        # implementation inspired by 
        # https://robotics.stackexchange.com/questions/109924/alternative-to-tf2-sensor-msgs-do-transform-cloud-in-ros-2humble-for-point-clo
        t = np.eye(4)
        q = transform.transform.rotation
        x = transform.transform.translation
        t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        t[:3, 3] = [x.x, x.y, x.z]

        print(len(pointcloud_coord))

        pointcloud_new = np.ones((len(pointcloud_coord), 4))
        pointcloud_new[:, :3] = pointcloud_coord
        pointcloud_new = np.matmul(t, pointcloud_new.T)
        print("DEBUG0")
        pointcloud_new[0, :] = np.divide(pointcloud_new[0, :], pointcloud_new[3, :])
        pointcloud_new[1, :] = np.divide(pointcloud_new[1, :], pointcloud_new[3, :])
        pointcloud_new[2, :] = np.divide(pointcloud_new[2, :], pointcloud_new[3, :])
        pointcloud_new = pointcloud_new.T

        pointcloud_rst = pointcloud
        pointcloud_rst[:, :3] = pointcloud_new[:, :3]

        print("DEBUG1")

        # for debug/visualization purpose
        if not header is None:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            pointcloud_map_header = header
            pointcloud_map_header.frame_id = "map"
            pointcloud_map = point_cloud2.create_cloud(pointcloud_map_header, fields, pointcloud_new[:, :3])
            self.pcl_debug_pub.publish(pointcloud_map)

        print("DEBUG2")

        return pointcloud_rst
    
    def _calc_pointcloud_group_vel(self, prev_ptcloud, curr_ptcloud, prev_pose, curr_pose, dt):
        # This function uses small threshold, distane based clustering to assign groups.
        # Then compare the assigned groups to previous groups to get pointcloud velocities.

        num_prev = len(prev_ptcloud)
        num_curr = len(curr_ptcloud)
        if num_prev == 0:
            complete_ptcloud = np.array([])
        else:
            complete_ptcloud = np.array([])
        return complete_ptcloud

    def _is_valid_pt(self, elem):
        # test if ring specified
        if not (self._ring_limit == -1):
            ring = elem[4]
            if not ring == self._ring_limit:
                return False

        # test if point distance passes max range
        if self._scan_max_range < 100:
            x = elem[0]
            y = elem[1]
            dist = np.sqrt(x*x + y*y)
            if dist > self._scan_max_range:
                return False

        return True
    
    def _copy_pointcloud(self):
        # Copy the point cloud from queue to an array for processing later

        self.pointcloud_history_np = list(self.pointcloud_history.queue)
        return
    
    def process_loop(self):
        # The main loop that performs:
        # 1. Grouping of point clouds
        # 2. Generate group representations
        # 3. Run predictions on representations
        # 4. Publish current and predicted representations

        while True:
            if not self.is_history_complete:
                loop_rate = self.create_rate(1, self.get_clock())
                loop_rate.sleep()
                continue
            try:
                self._copy_pointcloud()
            except KeyboardInterrupt:
                self.get_logger().info("Loop terminated")
                break
        return


def main():
    rclpy.init()
    scan_receiver = ScanReceiver()
    #process_thread = threading.Thread(target=scan_receiver.process_loop)
    #process_thread.start()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(scan_receiver, executor)
    except KeyboardInterrupt:
        #process_thread.join()
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
