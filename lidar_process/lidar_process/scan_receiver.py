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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from builtin_interfaces.msg import Duration
from cabot_msgs.msg import PoseLog

from . import pcl_to_numpy
from . import utils
from . import visualization

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data

from cabot_msgs.srv import LookupTransform

import open3d as o3d

class ScanReceiver(Node):

    def __init__(self):
        super().__init__('scan_receiver')

        state_update_callback_group = MutuallyExclusiveCallbackGroup()
        transform_lookup_callback_group = MutuallyExclusiveCallbackGroup()
        visualization_callback_group = MutuallyExclusiveCallbackGroup()
        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sensor_data_qos = qos_profile_sensor_data

        self.scan_sub = self.create_subscription(
            PointCloud2, 
            '/velodyne_points_cropped',
            self.scan_cb,
            qos_profile=sensor_data_qos,
            callback_group = state_update_callback_group
        )
        self.scan_sub = self.create_subscription(
            PoseLog, 
            '/cabot/pose_log',
            self.pose_cb,
            qos_profile=sensor_data_qos,
            callback_group = state_update_callback_group
        )
        
        self.pcl_debug_pub = self.create_publisher(
            PointCloud2,
            "/map_velodyne_points",
            10,
            callback_group = visualization_callback_group
        )

        self.entity_vis_pub = self.create_publisher(
            MarkerArray,
            "/vis_entities",
            10,
            callback_group = visualization_callback_group
        )

        self.lookup_transform_service = self.create_client(
            LookupTransform, 
            '/lookup_transform', 
            callback_group=transform_lookup_callback_group
        )


        self.pointcloud_header = None
        self.pointcloud = np.array([])
        self.pointcloud_prev = np.array([])
        self.pointcloud_complete = np.array([])

        self.pose = None
        self.prev_pose = None
        self.curr_pose = None
        self.curr_time = 0
        self.prev_time = 0

        self.namespace = self.declare_parameter('namespace', '').value
        self._ring_limit = self.declare_parameter('ring_limit', -1).value
        self._scan_max_range = self.declare_parameter('scan_max_range', 15).value
        self._history_window = self.declare_parameter('history_window', 8).value
        self._future_window = self.declare_parameter('future_window', 8).value
        # samples needed to be considered core points for DBSCAN clustering
        self._low_level_core_samples = self.declare_parameter('low_level_core_samples', 5).value
        # threshold values for DBSCAN
        # low level is used for denoising and calculating velocities
        # high level is used for pedestrian grouping
        self._low_level_pos_threshold = self.declare_parameter('low_level_pos_threshold', 0.5).value
        self._high_level_pos_threshold = self.declare_parameter('high_level_pos_threshold', 1).value
        self._high_level_vel_threshold = self.declare_parameter('high_level_vel_threshold', 1).value
        self._high_level_ori_threshold = self.declare_parameter('high_level_ori_threshold', 1).value

        self.pointcloud_history = queue.Queue(self._history_window)
        self.pointcloud_history_np = []  # list of numpys
        self.is_history_complete = False

        self.debug_visualiation = True

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
        
        # sync call end here
        result = future.result()
        if result.error.error > 0:
            raise Exception(result.error.error_string)
        return result.transform

    def scan_cb(self, msg):
        # Stores pointcloud when the node receives one.

        self.get_logger().info("pcl received")
        self.pointcloud_header = msg.header
        start_time = time.time()

        if self.pose is None:
            return

        self.curr_pose = self.pose
        self.curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = self.curr_time - self.prev_time
        self.prev_pose = self.curr_pose
        self.prev_time = self.curr_time
        if dt <= 0:
            return

        # Pointcloud format
        # (x, y, z, intensity, ring number [e.g. 0-15])
        # Pointcloud complete format
        # groups are low level groups used for estimating velocities
        # (x, y, z, intensity, ring, group id, group center x, group center y, vel_x, vel_y)
        parsed_pointcloud_1 = pcl_to_numpy.read_points_numpy(
            msg, 
            field_names=['x', 'y', 'z', 'intensity'], 
            skip_nans=True)
        parsed_pointcloud_2 = pcl_to_numpy.read_points_numpy(
            msg, 
            field_names=['ring'], 
            skip_nans=True)

        num_raw_pt = len(parsed_pointcloud_1)
        if num_raw_pt > 0:
            num_field_1 = len(parsed_pointcloud_1[0])
            num_field_2 = len(parsed_pointcloud_2[0])
            self.pointcloud = np.zeros((num_raw_pt, num_field_1 + num_field_2))
            self.pointcloud[:, :num_field_1] = parsed_pointcloud_1
            self.pointcloud[:, num_field_1:] = parsed_pointcloud_2
            self.pointcloud = self._filter_pointcloud(self.pointcloud)
        else:
            self.pointcloud = np.array([])

        # transform pointcloud to map frame
        lidar_to_map = self.lookup_transform('velodyne', 'map')
        transformed_cloud = self._do_transform(self.pointcloud, lidar_to_map)

        if len(self.pointcloud) > 0:
            self.pointcloud_complete = self._calc_pointcloud_group_vel(
                                                self.pointcloud_prev, 
                                                transformed_cloud, 
                                                self.prev_pose, 
                                                self.curr_pose,
                                                dt)
        else:
            self.pointcloud_complete = []
        self.pointcloud_prev = self.pointcloud_complete 

        if self.pointcloud_history.qsize() == self._history_window :
            self.is_history_complete = True
            # pop the oldest point cloud from the queue
            tmp = self.pointcloud_history.get(block=False)
        self.pointcloud_history.put(self.pointcloud, block=False)

        print("This callback runs: {} seconds".format(time.time() - start_time))
        
        return
    
    def pose_cb(self, msg):
        self.pose = msg.pose
        return
    
    def _filter_pointcloud(self, pointcloud):
        # This function prefilters pointclouds that do not meet the requirements
        if not (self._ring_limit == -1):
            condition = (pointcloud[:, 4] == self._ring_limit)
            pointcloud = pointcloud[condition == True]
        if self._scan_max_range < 100:
            condition = (np.linalg.norm(pointcloud[:, :2], axis=1) < self._scan_max_range)
            pointcloud = pointcloud[condition == True]
        return pointcloud
    
    def _do_transform(self, pointcloud, transform):
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
        if self.debug_visualiation:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            pointcloud_map_header = self.pointcloud_header
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
        print("dt: {}".format(dt))

        num_prev = len(prev_ptcloud)
        num_curr = len(curr_ptcloud)
        complete_ptcloud = np.zeros((num_curr, 10))
        complete_ptcloud[:, :5] = curr_ptcloud
        ptcloud_pos = curr_ptcloud[:, :3]
        # z coordinates are not used for now, so use _
        (group, 
         group_x, 
         group_y, 
         _, 
         unique_group, 
         unique_group_x, 
         unique_group_y, 
         _) = self._get_groups_and_centers(ptcloud_pos)
        complete_ptcloud[:, 5] = group
        complete_ptcloud[:, 6] = group_x
        complete_ptcloud[:, 7] = group_y

        # A bit ad hoc, but if the obsacle is too big
        # It is likely a static obstacle and should have 0 velocity
        for i, gp_index in enumerate(unique_group):
            gp_mask = (group == gp_index)
            gp_x = group_x[gp_mask]
            gp_y = group_y[gp_mask]
            range_x = np.max(gp_x) - np.min(gp_x)
            range_y = np.max(gp_y) - np.min(gp_y)
            large_obs_threshold = 3
            if (range_x > large_obs_threshold) or (range_y > large_obs_threshold):
                unique_group[i] = -1
                group[gp_mask] = -1
                complete_ptcloud[gp_mask, 5] = -1 # -1 is not noise anymore, it means large obstacles

        # Won't work if too mcuh time has passed
        # Set zero velocities is this happens
        too_much_time_threshold = 0.25 

        #If no prior pointclouds are available then leave the velocity columns 8 and 9 to be 0
        if (num_prev > 0) and (dt < too_much_time_threshold):
            # estimate velocities for low level groups and use that as velocities of pointclouds
            group_prev = prev_ptcloud[:, 5]
            group_x_prev = prev_ptcloud[:, 6]
            group_y_prev = prev_ptcloud[:, 7]

            # compress prev groups to get unique information
            unique_group_prev = np.unique(group_prev)
            num_prev_group = len(unique_group_prev)
            unique_group_x_prev = np.zeros(num_prev_group)
            unique_group_y_prev = np.zeros(num_prev_group)
            for i, l in enumerate(unique_group_prev):
                unique_group_x_prev[i] = group_x_prev[group_prev == l][0]
                unique_group_y_prev[i] = group_y_prev[group_prev == l][0]

            # get matching pairs from curr unique groups and prev unique groups
            # for each pair: estimate a velocity and propagate the velocity to point clouds
            new_group_dist_threshold = 1
            for i, gp_index in enumerate(unique_group):
                if gp_index != -1 :
                    gp_mask = (group == gp_index)

                    # Assume the group that was closest before was the prior group
                    # If too far, then it is likely a new group
                    gp_center_x = unique_group_x[i]
                    gp_center_y = unique_group_y[i]
                    
                    min_dist = np.inf
                    min_idx = None
                    for j in range(num_prev_group):
                        if unique_group_prev[j] != -1:
                            gp_prev_center_x = unique_group_x_prev[j]
                            gp_prev_center_y = unique_group_y_prev[j]
                            dist = np.sqrt((gp_center_x - gp_prev_center_x) ** 2 + 
                                        (gp_center_y - gp_prev_center_y) ** 2)
                            if dist < min_dist:
                                min_dist = dist
                                min_idx = j

                    if min_dist < new_group_dist_threshold:
                        min_prev_x = unique_group_x_prev[min_idx]
                        min_prev_y = unique_group_y_prev[min_idx]
                        group_vel_x = (gp_center_x - min_prev_x) / dt
                        group_vel_y = (gp_center_y - min_prev_y) / dt
                        complete_ptcloud[gp_mask, 5] = unique_group_prev[min_idx]
                        complete_ptcloud[gp_mask, 8] = group_vel_x
                        complete_ptcloud[gp_mask, 9] = group_vel_y
                    else:
                        # assign new unique group id
                        print("New id assigned: min_dist {}".format(min_dist))
                        complete_ptcloud[gp_mask, 5] = np.max(unique_group_prev) + gp_index + 1
        elif (dt > too_much_time_threshold):
            self.get_logger().info("Too much time has passed since last velodyne message received!")
        elif (num_prev == 0):
            self.get_logger().info("Previous pointcloud is empty!")

        # delete noise and large obstacles
        complete_ptcloud = complete_ptcloud[complete_ptcloud[:, 5] != -1]

        if self.debug_visualiation:
            entity_labels = np.unique(complete_ptcloud[:, 5])
            #print("Number of entities: {}".format(len(entity_labels)))
            #print(entity_labels)
            entity_idxes = complete_ptcloud[:, 5]
            marker_array = MarkerArray()
            marker_list = []
            for i, l in enumerate(entity_labels):
                entity_label = int(l)
                entity_x = complete_ptcloud[entity_idxes == l, 6][0]
                entity_y = complete_ptcloud[entity_idxes == l, 7][0]
                entity_vx = complete_ptcloud[entity_idxes == l, 8][0]
                entity_vy = complete_ptcloud[entity_idxes == l, 9][0]
                print([entity_vx, entity_vy])
                marker, text_marker, vel_marker = visualization.create_entity_marker_with_velocity(
                    entity_x, 
                    entity_y, 
                    entity_vx, 
                    entity_vy, 
                    entity_label, 
                    self.pointcloud_header, 
                    self.namespace)
                marker_list.append(marker)
                marker_list.append(text_marker)
                marker_list.append(vel_marker)
            marker_array.markers = marker_list
            self.entity_vis_pub.publish(marker_array)

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
        group = self._get_groups(
                    ptcloud, 
                    self._low_level_pos_threshold, 
                    self._low_level_core_samples
                    )
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
            group_x_unique = group_x_unique[1:]
            group_y_unique = group_y_unique[1:]
            group_z_unique = group_z_unique[1:]

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
