import os
import signal
import sys
import time
import copy
import threading
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from cabot_msgs.msg import PoseLog # type: ignore
from lidar_process_msgs.msg import Group, GroupArray, GroupTimeArray #type: ignore

from . import pcl_to_numpy
from . import utils
from . import visualization
from . import grouping
from .sgan import inference

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data

from cabot_msgs.srv import LookupTransform # type: ignore

import open3d as o3d

class ScanReceiver(Node):

    def __init__(self):
        super().__init__('scan_receiver')

        state_update_callback_group = MutuallyExclusiveCallbackGroup()
        transform_lookup_callback_group = MutuallyExclusiveCallbackGroup()
        visualization_callback_group = MutuallyExclusiveCallbackGroup()
        group_prediction_callback_group = MutuallyExclusiveCallbackGroup()
        # For very low frequency publish, not used for now
        #transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sensor_data_qos = qos_profile_sensor_data

        group_cb_timer_period = 0.2

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
        self.group_pred_pub = self.create_publisher(
            GroupTimeArray,
            "/group_predictions",
            10,
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
        self.group_vis_pub = self.create_publisher(
            MarkerArray,
            "/vis_groups",
            10,
            callback_group=visualization_callback_group
        )

        self.lookup_transform_service = self.create_client(
            LookupTransform, 
            '/lookup_transform', 
            callback_group=transform_lookup_callback_group
        )

        self.group_generation_timer = self.create_timer(
            group_cb_timer_period,
            self.group_gen_cb,
            callback_group = group_prediction_callback_group
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
        if not (self._history_window == 8):
            self.get_logger().error("Sorry, only support history length 8 now")
            sys.exit(0)
        if not (self._future_window == 8 or self._future_window == 12):
            self.get_logger().error("Sorry, only support future length 8 or 12 now")
            sys.exit(0)
        # samples needed to be considered core points for DBSCAN clustering
        self._low_level_core_samples = self.declare_parameter('low_level_core_samples', 5).value
        # threshold values for DBSCAN
        # low level is used for denoising and identifying entities
        # high level is used for pedestrian grouping
        self._low_level_pos_threshold = self.declare_parameter('low_level_pos_threshold', 0.5).value
        self._high_level_pos_threshold = self.declare_parameter('high_level_pos_threshold', 2.0).value
        self._high_level_vel_threshold = self.declare_parameter('high_level_vel_threshold', 1.0).value
        self._high_level_ori_threshold = self.declare_parameter('high_level_ori_threshold', 30.0).value
        self._high_level_ori_threshold = self._high_level_ori_threshold / 180 * np.pi
        self._static_threshold = self.declare_parameter('static_threshold', 0.5).value
        # parameters related to tracking of entities
        self._max_tracking_time = self.declare_parameter('max_tracking_time', 0.25).value
        self._max_tracking_dist = self.declare_parameter('max_tracking_dist', 1.0).value
        self._large_obs_size = self.declare_parameter('large_obs_size', 2.0).value
        # parameters related to the inputs to the prediction model
        self._max_queue_size = self.declare_parameter('max_queue_size', 50).value
        self._history_dt = self.declare_parameter('history_dt', 0.4).value
        # parameters related to the shapes of the group representations
        self._shape_increments = self.declare_parameter('shape_increments', 16).value
        self._shape_scale = self.declare_parameter('shape_scale', 0.354163).value
        self._shape_offset = self.declare_parameter('shape_offset', 1.0).value

        model_path = os.path.join(get_package_share_directory('lidar_process'),  # this package name
                                  "sgan-models", 
                                  "univ_" + str(self._future_window) + "_model.pt")
        self.group_pred_model = inference.SGANInference(model_path)

        self.pointcloud_history = utils.SimpleQueue(self._max_queue_size)

        self.debug_visualiation = True

        return
    
    def lookup_transform(self, source, target):
        # Look up and return tf transformation from source frame to target frame

        req = LookupTransform.Request()
        req.target_frame = target
        req.source_frame = source
        req.time = self.pointcloud_header.stamp
        if not self.lookup_transform_service.wait_for_service(timeout_sec=1.0):
            raise Exception("lookup transform service is not available")
        
        while True:
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
                # If transform data is not available yet (error 3)
                if result.error.error == 3:
                    continue
                raise Exception(result.error.error_string)
            else:
                return result.transform

    def scan_cb(self, msg):
        # Processes and stores pointcloud when the node receives one.

        #self.get_logger().info("pcl received")
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
        # (x, y, z, intensity, ring, group id, group center x, group center y, timestamp)
        # Break down into 2 parts to read 2 different field type values in pointcloud2
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
            self.pointcloud_complete = self._generate_track_entities(
                                                self.pointcloud_prev, 
                                                transformed_cloud, 
                                                dt)
        else:
            self.pointcloud_complete = []
        self.pointcloud_prev = self.pointcloud_complete 

        if self.pointcloud_history.is_full() :
            self.is_history_complete = True
            # pop the oldest point cloud from the queue
            tmp = self.pointcloud_history.dequeue()
        self.pointcloud_history.enqueue(self.pointcloud_complete)

        self.get_logger().info("Lidar scan callback runs: {} seconds".format(time.time() - start_time))
        
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
    
    def _generate_track_entities(self, prev_ptcloud, curr_ptcloud, dt):
        # This function uses small threshold, distane based clustering to assign groups as entities.
        # Then compare the assigned entities to previous entities to get tracking info (if same entity ids).
        #
        # returns pointcloud with low level entity (pedestrian/object) information and timestamps

        assert(dt > 0)

        #print("----------Pose & time info----------")
        #print("dt: {}".format(dt))

        num_prev = len(prev_ptcloud)
        num_curr = len(curr_ptcloud)
        complete_ptcloud = np.zeros((num_curr, 9))
        complete_ptcloud[:, :5] = curr_ptcloud
        complete_ptcloud[:, 8] = self.curr_time
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
        large_obs_size = self._large_obs_size
        for i, gp_index in enumerate(unique_group):
            gp_mask = (group == gp_index)
            gp_x = complete_ptcloud[gp_mask, 0]
            gp_y = complete_ptcloud[gp_mask, 1]
            range_x = np.max(gp_x) - np.min(gp_x)
            range_y = np.max(gp_y) - np.min(gp_y)
            max_obs_size = np.sqrt(range_x ** 2 + range_y ** 2)
            if (max_obs_size > large_obs_size):
                unique_group[i] = -1
                group[gp_mask] = -1
                complete_ptcloud[gp_mask, 5] = -1 # -1 is not just noise, also large obstacles

        # Won't work if dt passed max tracking time
        # Set entities to be new entities if this happens
        max_tracking_time = self._max_tracking_time

        #If no prior pointclouds are available then leave the velocity columns 8 and 9 to be 0
        if (num_prev > 0) and (dt < max_tracking_time):
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
            # for each pair: obtain the same entity id and assign to point clouds
            max_tracking_dist = self._max_tracking_dist
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

                    if min_dist < max_tracking_dist:
                        complete_ptcloud[gp_mask, 5] = unique_group_prev[min_idx]
                    else:
                        # assign new unique group id
                        #print("New id assigned: min_dist {}".format(min_dist))
                        complete_ptcloud[gp_mask, 5] = np.max(unique_group_prev) + gp_index + 1
        elif (dt > max_tracking_time):
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
                marker, text_marker = visualization.create_entity_marker(
                    entity_x, 
                    entity_y, 
                    entity_label, 
                    self.pointcloud_header, 
                    self.namespace)
                marker_list.append(marker)
                marker_list.append(text_marker)
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
        group = grouping.get_groups(
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
    
    def pose_cb(self, msg):
        # Stores the most up to date pose information
        self.pose = msg.pose
        return
    
    def group_gen_cb(self):
        # Performs grouping and generates input for the group prediction model
        # 1. Align point clouds at specified time intervals and estimate velocities
        # 2. Perform pedestrian grouping
        # 3. Identify the 3 key points for groups
        # 4. Apply proxemics
        # 5. Perform prediction

        if not(self.pointcloud_history.is_full()):
            return
        start_time = time.time()

        curr_robot_pose = self.pose
        curr_robot_x = curr_robot_pose.position.x
        curr_robot_y = curr_robot_pose.position.y
        curr_robot_pose = [curr_robot_x, curr_robot_y]

        # pcl_history is from old to new
        # pcl format:
        # (x, y, z, intensity, ring, group id, group center x, group center y, timestamp)
        pcl_history = copy.copy(self.pointcloud_history._items)
        if (len(pcl_history[0]) == 0):
            self.get_logger().warn("No pointclouds at current time.")
            return
        entities_history = self._align_pointclouds(pcl_history)

        # concatenate all entities at the current time
        curr_pos_array, curr_vel_array, curr_entities_id = self._concatenate_entities(entities_history, 0)
        curr_labels = grouping.pedestrian_grouping(
            curr_pos_array, 
            curr_vel_array, 
            self._high_level_pos_threshold, 
            self._high_level_vel_threshold,  
            self._high_level_ori_threshold,
            static_threshold=self._static_threshold)
        unique_entities = np.unique(curr_entities_id)
        entity_to_group = {}
        for i, l in enumerate(unique_entities):
            idxes = np.where(curr_entities_id == l)
            idx = idxes[0][0]
            gp = curr_labels[idx]
            entity_to_group[l] = gp
        
        # clustering is only performed at current time, prior groups are just tracing entities
        # only edge points are needed to perform future predictions
        sub_start_time = time.time()
        group_keypoint_sequences = {}
        for i in range(self._history_window - 1, -1, -1):  # the newer the later (currently the older the later)
            if i == 0:
                pos_array = curr_pos_array
                labels = curr_labels
            else:
                pos_array, _, entities_id = self._concatenate_entities(entities_history, i)
                labels = [entity_to_group[id] for id in entities_id]
            labels = np.array(labels)

            unique_groups = np.unique(labels)
            for g in unique_groups:
                group_condition = (labels == g)
                group_pos = pos_array[group_condition]
                left_idx, center_idx, right_idx = grouping.identify_edge_points(
                    group_pos, 
                    curr_robot_pose)
                if not (g in group_keypoint_sequences.keys()):
                    group_keypoint_sequences[g] = [[group_pos[left_idx], group_pos[center_idx], group_pos[right_idx]]]
                else:
                    group_keypoint_sequences[g].append([group_pos[left_idx], group_pos[center_idx], group_pos[right_idx]])
        
        # prepare the inputs to the trjectory prediction model and make predictions
        num_groups = len(group_keypoint_sequences)
        group_representations = GroupTimeArray()
        group_representations.group_sequences = []

        if num_groups > 0:
            group_pred_inputs = np.zeros((num_groups * 3, self._history_window, 2))
            for i, g in enumerate(group_keypoint_sequences.keys()):
                group = group_keypoint_sequences[g]
                assert(len(group) == self._history_window)
                group_pred_inputs[(i*3):((i+1)*3), :, :] = np.transpose(np.array(group), (1, 0, 2))
            
            group_futures = self.group_pred_model.evaluate(group_pred_inputs)
            group_complete_futures = np.zeros((num_groups * 3, self._future_window + 1, 2))
            group_complete_futures[:, 0, :] = group_pred_inputs[:, -1, :]
            group_complete_futures[:, 1:, :] = group_futures
            group_complete_velocities = (group_complete_futures[:, 1:, :] - group_complete_futures[:, :-1, :]) / self._history_dt

            for i in range(self._future_window):
                group_vertices = grouping.vertices_from_edge_pts(
                    curr_robot_pose, 
                    group_complete_futures[:, i, :], 
                    group_complete_velocities[:, i, :],
                    increments=self._shape_increments,
                    const=self._shape_scale,
                    offset=self._shape_offset)
                curr_groups = GroupArray()
                curr_groups.groups = []
                for i in range(len(group_vertices)):
                    group = group_vertices[i]
                    group_rep = Group()
                    group_rep.left.x = group['left'][0]
                    group_rep.left.y = group['left'][1]
                    group_rep.center.x = group['center'][0]
                    group_rep.center.y = group['center'][1]
                    group_rep.right.x = group['right'][0]
                    group_rep.right.y = group['right'][1]
                    group_rep.left_offset.x = group['left_offset'][0]
                    group_rep.left_offset.y = group['left_offset'][1]
                    group_rep.right_offset.x = group['right_offset'][0]
                    group_rep.right_offset.y = group['right_offset'][1]
                    curr_groups.groups.append(group_rep)
                group_representations.group_sequences.append(curr_groups)

        self.group_pred_pub.publish(group_representations)
               
        print("Time gen rep: {}".format(time.time() - sub_start_time))

        if self.debug_visualiation:
            group_vis_markers = MarkerArray()
            group_markers_list = []
            curr_groups = group_representations.group_sequences[0].groups
            header = Header()
            header.frame_id = "map"
            header.stamp = self.get_clock().now().to_msg()
            ns = self.namespace 
            for i in range(len(curr_groups)):
                group = curr_groups[i]
                group_markers_list.extend(visualization.create_group_marker(group, i, header, ns))
            group_vis_markers.markers = group_markers_list
            self.group_vis_pub.publish(group_vis_markers)

        # Publish group_representations

        self.get_logger().info("Group prediction callback runs: {} seconds".format(time.time() - start_time))

        return
    
    def _align_pointclouds(self, pcl_history):
        # Aligns the pointclouds at select intervals (specified by params) among recorded pointclouds history
        # Interpolations are performed for pointclouds between time intervals
        # If there is missing history information, propogation is used to fill in

        current_pcl = pcl_history[-1]
        if len(current_pcl) == 0:
            return []
        current_time = current_pcl[0, 8]

        # extract entities from current pointcloud and trace backwards
        # interpolation used from neighboring entity pointclouds
        # if history incomplete, perform back propogation
        # velocities also calculated here
        entities = current_pcl[:, 5]
        #entities_x = current_pcl[:, 6]
        #entities_y = current_pcl[:, 7]

        unique_entities = np.unique(entities)
        #num_entities = len(unique_entities)
        #unique_entities_x = np.zeros(num_entities)
        #unique_entities_y = np.zeros(num_entities)
        #for i, l in enumerate(unique_entities):
        #    unique_entities_x[i] = entities_x[entities == l][0]
        #    unique_entities_y[i] = entities_y[entities == l][0]

        # we collect info on history + 1 pointcouds to get velocities on history pointclouds
        entities_history = []
        for i, l in enumerate(unique_entities):
            current_entity = current_pcl[entities == l, :]
            entity_history = [current_entity]
            time_pointer = self._max_queue_size - 2
            propogation_step = 0
            for j in range(self._history_window):
                time_target = current_time - (j + 1) * self._history_dt
                pcl = pcl_history[time_pointer]
                while (len(pcl) > 0) and (pcl[0, 8] > time_target):
                    time_pointer -= 1
                    pcl = pcl_history[time_pointer]
                if (len(pcl) == 0) or (not l in pcl[:, 5]):
                    propogation_step = self._history_window - j
                    break
                else:
                    #interpolation
                    pcl_prev = pcl
                    pcl_next = pcl_history[time_pointer + 1]
                    entity_prev = pcl_prev[pcl_prev[:, 5] == l, :]
                    entity_next = pcl_next[pcl_next[:, 5] == l, :]
                    entity_interp = self._interpolate_pointclouds(entity_prev, entity_next, time_target)
                    entity_history.append(entity_interp)
            
            # calculate velocities and propogation
            # in final form, each entity will have (x, y, vx, vy, o)
            entity_history_with_vel = []
            if not(len(entity_history) == (self._history_window - propogation_step + 1)):
                self.get_logger().error("Code error: entity history length an propogation step mismatch!")
                sys.exit(0)
            for j in range(self._history_window - propogation_step):
                entity = entity_history[j]
                num_pts = len(entity)
                entity_with_vel = np.zeros((num_pts, 6))
                entity_prev = entity_history[j + 1]
                entity_with_vel[:, 0] = entity[:, 0]
                entity_with_vel[:, 1] = entity[:, 1]
                vx = (entity[0, 6] - entity_prev[0, 6]) / self._history_dt
                vy = (entity[0, 7] - entity_prev[0, 7]) / self._history_dt
                entity_with_vel[:, 2] = vx
                entity_with_vel[:, 3] = vy
                entity_with_vel[:, 4] = np.arctan2(vy, vx)
                entity_with_vel[:, 5] = l
                entity_history_with_vel.append(entity_with_vel)
            if propogation_step == self._history_window:
                offset_vx = 0
                offset_vy = 0
            else:
                oldest_entity = entity_history_with_vel[-1]
                offset_vx = oldest_entity[0, 2]
                offset_vy = oldest_entity[0, 3]
            entity = entity_history[-1]
            num_pts = len(entity)
            for j in range(propogation_step):
                entity_with_vel = np.zeros((num_pts, 6))
                if j == 0:
                    entity_with_vel[:, 0] = entity[:, 0]
                    entity_with_vel[:, 1] = entity[:, 1]
                else:
                    entity_with_vel[:, 0] = prev_entity[:, 0] - offset_vx * self._history_dt
                    entity_with_vel[:, 1] = prev_entity[:, 1] - offset_vy * self._history_dt
                entity_with_vel[:, 2] = offset_vx
                entity_with_vel[:, 3] = offset_vx
                entity_with_vel[:, 4] = np.arctan2(offset_vy, offset_vx)
                entity_with_vel[:, 5] = l
                entity_history_with_vel.append(entity_with_vel)
                prev_entity = entity_with_vel

            entities_history.append(entity_history_with_vel)

        return entities_history
    
    def _interpolate_pointclouds(self, pcl_1, pcl_2, curr_time, front_base=True):
        # Interpolates 2 point clouds
        # A very heuristics-based method but should work well
        # Because error tolerance is high for mismatching pairs etc.
        # 1. use entity centers to perform offset
        # 2. use nearest neighbors to get matching pairs
        # 3. interpolate
        # The interpolation is based on the 2nd pointcloud
        # If switching needed, then set front_base = True

        if front_base == True:
            tmp = pcl_1
            pcl_1 = pcl_2
            pcl_2 = tmp

        num_pts1 = len(pcl_1)
        num_pts2 = len(pcl_2)
        if (num_pts1 == 0) and (num_pts2 == 0):
            self.get_logger().warn("Both pointclouds are blank for interpolation!")
            return []
        if num_pts1 == 0:
            return pcl_2
        if num_pts2 == 0:
            return pcl_1
        
        # displacement by center offsets
        center_1x = pcl_1[0, 6]
        center_1y = pcl_1[0, 7]
        center_2x = pcl_2[0, 6]
        center_2y = pcl_2[0, 7]
        pcl_1[:, 0] = pcl_1[:, 0] + (center_2x - center_1x)
        pcl_1[:, 1] = pcl_1[:, 1] + (center_2y - center_1y)

        # get pairs by dearest neighbor
        mesh_1x, mesh_2x = np.meshgrid(pcl_1[:, 0], pcl_2[:, 0])
        mesh_1y, mesh_2y = np.meshgrid(pcl_1[:, 1], pcl_2[:, 1])
        mesh_dist = np.sqrt(np.square(mesh_1x - mesh_2x) + np.square(mesh_1y - mesh_2y))
        min_dist_idxes = np.argmin(mesh_dist, axis=1)
        pcl_1 = pcl_1[min_dist_idxes, :]

        # interpolate
        time_1 = pcl_1[0, 8]
        time_2 = pcl_2[0, 8]
        pcl_interp = pcl_2 + (pcl_1 - pcl_2) / (time_1 - time_2) * (curr_time - time_2)
        # preserve ring number, id, time
        pcl_interp[:, 4] = pcl_2[:, 4]
        pcl_interp[:, 5] = pcl_2[0, 5]
        pcl_interp[:, 8] = curr_time

        return pcl_interp
    
    def _concatenate_entities(self, entities_history, time):
        # This function concatenates the positions and velocities of the entities at a given time
        # Note the entity ids output is a numpy array
        num_entities = len(entities_history)

        if (num_entities > 0) and (time >= len(entities_history[0])):
            self.get_logger().error("Time chosen is longer than the hsitory time window of entities")
            sys.exit(0)

        """
        pos_array = []
        vel_array = []
        ent_array = []
        for i in range(num_entities):
            entity = entities_history[i][time]
            entity_pos = entity[:, :2]
            entity_vel = entity[:, 2:4]
            entity_id = entity[:, 5]
            pos_array = pos_array + entity_pos.tolist()
            vel_array = vel_array + entity_vel.tolist()
            ent_array = ent_array + entity_id.tolist()

        return np.array(pos_array), np.array(vel_array), np.array(ent_array)
        """

        pos_entities = tuple(entity[time][:, :2] for entity in entities_history)
        vel_entities = tuple(entity[time][:, 2:4] for entity in entities_history)
        ent_entities = tuple(entity[time][:, 5] for entity in entities_history)

        if num_entities == 0:
            return np.array([]), np.array([]), np.array([])
        else:
            return np.concatenate(pos_entities, axis=0), \
                np.concatenate(vel_entities, axis=0), \
                np.concatenate(ent_entities, axis=0)


def main():
    rclpy.init()
    scan_receiver = ScanReceiver()

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
