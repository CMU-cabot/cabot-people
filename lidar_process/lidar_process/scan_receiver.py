import signal
import sys
import queue
import time
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
from . import pcl_to_numpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from cabot_msgs.srv import LookupTransform

import open3d as o3d

class ScanReceiver(Node):

    def __init__(self):
        super().__init__('scan_receiver')

        state_update_callback_group = MutuallyExclusiveCallbackGroup()
        transform_lookup_callback_group = MutuallyExclusiveCallbackGroup()
        visualization_callback_group = MutuallyExclusiveCallbackGroup()
        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.scan_sub = self.create_subscription(
            PointCloud2, 
            '/velodyne_points_cropped',
            self.scan_cb,
            #qos_profile=transient_local_qos,
            10,
            callback_group = state_update_callback_group
        )
        self.scan_sub = self.create_subscription(
            PoseLog, 
            '/cabot/pose_log',
            self.pose_cb,
            #qos_profile=transient_local_qos,
            10,
            callback_group = state_update_callback_group
        )
        
        self.pcl_debug_pub = self.create_publisher(
            PointCloud2,
            "/map_velodyne_points",
            #qos_profile=transient_local_qos,
            10,
            callback_group = visualization_callback_group
        )

        self.lookup_transform_service = self.create_client(
            LookupTransform, 
            '/lookup_transform', 
            #qos_profile=transient_local_qos,
            callback_group=transform_lookup_callback_group
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
        # samples needed to be considered core points for DBSCAN clustering
        self._low_level_core_samples = self.declare_parameter('low_level_core_samples', 5).value
        # threshold values for DBSCAN
        # low level is used for denoising and calculating velocities
        # high level is used for pedestrian grouping
        self._low_level_pos_threshold = self.declare_parameter('low_level_pos_threshold', 0.25).value
        self._high_level_pos_threshold = self.declare_parameter('high_level_pos_threshold', 1).value
        self._high_level_vel_threshold = self.declare_parameter('high_level_vel_threshold', 1).value
        self._high_level_ori_threshold = self.declare_parameter('high_level_ori_threshold', 1).value

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

        self.get_logger().info("pcl received")
        start_time = time.time()

        if self.pose is None:
            return

        self.curr_pose = self.pose
        self.curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = self.curr_time - self.prev_time

        # Pointcloud format
        # (x, y, z, intensity, ring number [e.g. 0-15])
        # Pointcloud complete format
        # (x, y, z, intensity, ring, group id, group center x, group center y, vel_x, vel_y)
        parsed_pointcloud_1 = pcl_to_numpy.read_points_numpy(
            msg, 
            field_names=['x', 'y', 'z', 'intensity'], 
            skip_nans=True)
        parsed_pointcloud_2 = pcl_to_numpy.read_points_numpy(
            msg, 
            field_names=['ring'], 
            skip_nans=True)

        tmp_start_time = time.time()
        num_raw_pt = len(parsed_pointcloud_1)
        if num_raw_pt > 0:
            num_field_1 = len(parsed_pointcloud_1[0])
            num_field_2 = len(parsed_pointcloud_2[0])
            self.pointcloud = np.zeros((num_raw_pt, num_field_1 + num_field_2))
            self.pointcloud[:, :num_field_1] = parsed_pointcloud_1
            self.pointcloud[:, num_field_1:] = parsed_pointcloud_2
            #for i, elem in enumerate(parsed_pointcloud):
            #    self.pointcloud[i, :] = list(elem)
            self.pointcloud = self._filter_pointcloud(self.pointcloud)

            # old, much slower ways to parse point clouds
            """
            for i, elem in enumerate(parsed_pointcloud):
                self.pointcloud[i, :] = list(elem)
            self.pointcloud = self._filter_pointcloud(self.pointcloud)
            """
            """
            self.get_logger().info(f"type(parsed_pointcloud)={type(parsed_pointcloud[0])}")
            self.pointcloud = parsed_pointcloud
            points_array = np.array(parsed_pointcloud)
            print("points_array=", points_array)
            print("self.pointcloud=", self.pointcloud)
            self.pointcloud = self._filter_pointcloud(self.pointcloud)
            """
            """
            self.pointcloud = np.zeros((num_raw_pt, len(parsed_pointcloud[0])))
            validation_array = np.zeros(num_raw_pt)
            for i, elem in enumerate(parsed_pointcloud):
                self.pointcloud[i, :] = list(elem)
                if self._is_valid_pt(elem):
                    validation_array[i] = 1
            self.pointcloud = self.pointcloud[validation_array == 1]
            """
        else:
            self.pointcloud = np.array([])
        print(time.time() - tmp_start_time)

        # transform pointcloud to map frame
        lidar_to_map = self.lookup_transform('velodyne', 'map')
        transformed_cloud = self._do_transform(self.pointcloud, lidar_to_map, msg.header)

        print("--------pcl debug--------")
        print(len(self.pointcloud))
        if len(self.pointcloud) > 0:
            print(self.pointcloud[0])
            print(transformed_cloud[0])

            print("stuck here 1")
            self.pointcloud_complete = self._calc_pointcloud_group_vel(
                                                self.pointcloud_prev, 
                                                transformed_cloud, 
                                                self.prev_pose, 
                                                self.curr_pose,
                                                dt)
            print("not stuck here 1")
        else:
            self.pointcloud_complete = []
        self.pointcloud_prev = self.pointcloud_complete 
        self.prev_pose = self.curr_pose
        self.prev_time = self.curr_time

        if self.pointcloud_history.qsize() == self._history_window :
            self.is_history_complete = True
            # pop the oldest point cloud from the queue
            tmp = self.pointcloud_history.get(block=False)
        print("stuck here 2")
        self.pointcloud_history.put(self.pointcloud, block=False)
        print("not stuck here 2")

        print("This callback runs: {} seconds".format(time.time() - start_time))
        
        return
    
    def pose_cb(self, msg):
        self.pose = msg.pose
        return
    
    def _filter_pointcloud(self, pointcloud):
        if not (self._ring_limit == -1):
            condition = (pointcloud[:, 4] == self._ring_limit)
            pointcloud = pointcloud[condition == True]
        if self._scan_max_range < 100:
            condition = (np.linalg.norm(pointcloud[:, :2], axis=1) < self._scan_max_range)
            pointcloud = pointcloud[condition == True]
        return pointcloud
    
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

        print("Number of point clouds: {}".format(len(pointcloud_coord)))

        pointcloud_new = np.ones((len(pointcloud_coord), 4))
        pointcloud_new[:, :3] = pointcloud_coord
        pointcloud_new = np.matmul(t, pointcloud_new.T)
        pointcloud_new[0, :] = np.divide(pointcloud_new[0, :], pointcloud_new[3, :])
        pointcloud_new[1, :] = np.divide(pointcloud_new[1, :], pointcloud_new[3, :])
        pointcloud_new[2, :] = np.divide(pointcloud_new[2, :], pointcloud_new[3, :])
        pointcloud_new = pointcloud_new.T

        pointcloud_rst = np.copy(pointcloud)
        pointcloud_rst[:, :3] = pointcloud_new[:, :3]

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

        return pointcloud_rst
    
    def _calc_pointcloud_group_vel(self, prev_ptcloud, curr_ptcloud, prev_pose, curr_pose, dt):
        # This function uses small threshold, distane based clustering to assign groups.
        # Then compare the assigned groups to previous groups to get pointcloud velocities.
        #
        # returns pointcloud with low level group (pedestrian/object) information
        # and estimated velocities

        assert(dt > 0)

        print("----------Pose & time info----------")
        print(prev_pose)
        print(curr_pose)
        print(dt)

        num_prev = len(prev_ptcloud)
        num_curr = len(curr_ptcloud)
        complete_ptcloud = np.zeros((num_curr, 10))
        complete_ptcloud[:, :5] = curr_ptcloud
        ptcloud_pos = curr_ptcloud[:, :3]
        # z coordinates are not used for now, so use _
        print("stuck here 3")
        (group, 
         group_x, 
         group_y, 
         _, 
         unique_group, 
         unique_group_x, 
         unique_group_y, 
         _) = self._get_groups_and_centers(ptcloud_pos)
        print("not stuck here 3")
        complete_ptcloud[:, 5] = group
        complete_ptcloud[:, 6] = group_x
        complete_ptcloud[:, 7] = group_y

        # delete noise
        complete_ptcloud = complete_ptcloud[group != -1]

        #If no prior pointclouds are available then leave the velocity columns 8 and 9 to be 0
        if num_prev > 0:
            # estimate velocities for low level groups and use that as velocities of pointclouds
            group_prev = prev_ptcloud[:, 5]
            group_x_prev = prev_ptcloud[:, 6]
            group_y_prev = prev_ptcloud[:, 7]
            num_prev_group = len(group_prev)

            # compress prev groups to get unique information
            unique_group_prev = np.unique(group_prev)
            unique_group_x_prev = np.zeros(num_prev_group)
            unique_group_y_prev = np.zeros(num_prev_group)
            for i, l in enumerate(unique_group_prev):
                unique_group_x_prev[i] = group_x_prev[group_prev == l][0]
                unique_group_y_prev[i] = group_y_prev[group_prev == l][0]

            # get matching pairs from curr unique groups and prev unique groups
            # for each pair: estimate a velocity and propagate the velocity to point clouds
            for i, gp_index in enumerate(unique_group):
                gp_mask = (group == gp_index)
                gp_x = group_x[gp_mask]
                gp_y = group_y[gp_mask]
                range_x = np.max(gp_x) - np.min(gp_x)
                range_y = np.max(gp_y) - np.min(gp_y)
                # A bit ad hoc, but if the obsacle is too big
                # It is likely a static obstacle and should have 0 velocity
                large_obs_threshold = 5
                if (range_x > large_obs_threshold) or (range_y > large_obs_threshold):
                    continue

                # Assume the group that was closest before was the prior group
                # If too far, then it is likely a new group
                gp_center_x = unique_group_x[i]
                gp_center_y = unique_group_y[i]
                new_group_dist_threshold = 1
                min_dist = np.inf
                min_prev_x = None
                min_prev_y = None
                for j in range(len(unique_group_prev)):
                    gp_prev_center_x = unique_group_x_prev[j]
                    gp_prev_center_y = unique_group_y_prev[j]
                    dist = np.sqrt((gp_center_x - gp_prev_center_x) ** 2 + 
                                   (gp_center_y - gp_prev_center_y) ** 2)
                    if dist < min_dist:
                        min_dist = dist
                        min_prev_x = gp_prev_center_x
                        min_prev_y = gp_prev_center_y

                if min_dist < new_group_dist_threshold:
                    group_vel_x = (gp_center_x - min_prev_x) / dt
                    group_vel_y = (gp_center_y - min_prev_y) / dt
                    complete_ptcloud[gp_mask, 8] = group_vel_x
                    complete_ptcloud[gp_mask, 9] = group_vel_y

        return complete_ptcloud
    
    def _get_groups_and_centers(self, ptcloud):
        # performs low level clustering to obtains small clusters around objects/pedestrians
        # this is used to estimate the speed of the objects/pedestrians
        #
        # returns groups ids (or pedestrian/object ids) and the coordinates of the cluster centers
        # size is the same as the number of points in ptcloud
        #
        # also returns unique group ids (labels), and coordinates
        #
        # Note: very heuristics based, may not be a great pedestrian detector.
        num_pts = len(ptcloud)
        group_x = np.zeros(num_pts)
        group_y = np.zeros(num_pts)
        group_z = np.zeros(num_pts)
        print("stuck here 4")
        print(num_pts)
        group = self._get_groups(
                    ptcloud, 
                    self._low_level_pos_threshold, 
                    self._low_level_core_samples
                    )
        print("not stuck here 4")
        labels = np.unique(group)
        num_labels = len(labels)
        group_x_unique = np.zeros(num_labels)
        group_y_unique = np.zeros(num_labels)
        group_z_unique = np.zeros(num_labels)

        for i, l in enumerate(labels):
            # ignore noise (with group labels -1)
            if l == -1:
                continue
            group_x_val = np.mean(ptcloud[group == l, 0])
            group_y_val = np.mean(ptcloud[group == l, 1])
            group_z_val = np.mean(ptcloud[group == l, 2])
            group_x_unique[i] = group_x_val
            group_y_unique[i] = group_y_val
            group_z_unique[i] = group_z_val
            group_x[group == l] = group_x_val
            group_y[group == l] = group_y_val
            group_z[group == l] = group_z_val

        if labels[0] == -1:
            labels = labels[1:]

        return (group, 
            group_x, 
            group_y, 
            group_z, 
            labels, 
            group_x_unique, 
            group_y_unique, 
            group_z_unique)
    
    def _get_groups(self, data, threshold, core_samples):
        # performs DBSCAN and clusters according to threshold and 
        # number of sample points for the cluster cores
        #
        # returns the indexed labels with the same size as data
        clusters = DBSCAN(eps=threshold, min_samples=core_samples).fit(data)
        return clusters.labels_
    
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
    except Exception as err:
        scan_receiver.get_logger().error(err)

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
