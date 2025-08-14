# Copyright (c) 2021, 2022  IBM Corporation and Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from abc import ABCMeta, abstractmethod
from collections import defaultdict
import copy
import itertools
import math

from diagnostic_updater import Updater
from diagnostic_updater import HeaderlessTopicDiagnostic
from diagnostic_updater import FrequencyStatusParam
from geometry_msgs.msg import PointStamped, Vector3Stamped
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import rclpy.node
from rclpy.duration import Duration
from people_msgs.msg import People, Person
from tf_transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_point, do_transform_vector3
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from track_people_msgs.msg import TrackedBoxes


class AbsTrackPeople(rclpy.node.Node):
    __metaclass__ = ABCMeta

    def __init__(self, n_colors=100):
        super().__init__('track_people_py')

        self.n_colors = n_colors
        self.list_colors = plt.cm.hsv(np.linspace(0, 1, n_colors)).tolist()  # list of colors to assign to each track for visualization
        np.random.shuffle(self.list_colors)  # shuffle colors

        self.output_frame = self.declare_parameter('output_frame', 'map').value
        self.iou_threshold = self.declare_parameter('iou_threshold', 0.01).value
        self.iou_circle_size = self.declare_parameter('iou_circle_size', 0.5).value
        self.kf_init_var = self.declare_parameter('kf_init_var', 1.0).value
        self.kf_process_var = self.declare_parameter('kf_process_var', 1.0).value
        self.kf_measure_var = self.declare_parameter('kf_measure_var', 1.0).value
        # number of frames to start publish people topic
        self.minimum_valid_track_observe = self.declare_parameter('minimum_valid_track_observe', 5).value
        # duration (seconds) to remove an inactive track
        self.duration_inactive_to_remove = self.declare_parameter('duration_inactive_to_remove', 2.0).value
        # duration (seconds) to stop publishing an inactive track in people topic
        self.duration_inactive_to_stop_publish = self.declare_parameter('duration_inactive_to_stop_publish', 0.2).value

        self.detected_boxes_sub = self.create_subscription(TrackedBoxes, 'people/detected_boxes', self.detected_boxes_cb, 10)
        self.people_pub = self.create_publisher(People, 'people', 10)
        self.vis_marker_array_pub = self.create_publisher(MarkerArray, 'people/tracking_visualization', 10)

        self.updater = Updater(self)
        target_fps = self.declare_parameter('target_fps', 10.0).value
        diagnostic_name = self.declare_parameter('diagnostic_name', "PeopleTrack").value
        self.htd = HeaderlessTopicDiagnostic(diagnostic_name, self.updater,
                                             FrequencyStatusParam({'min': target_fps, 'max': target_fps}, 0.2, 2))

        self.stationary_detect_threshold_duration_ = self.declare_parameter('stationary_detect_threshold_duration', 1.0).value
        self.stationary_detect_threshold_velocity_ = self.declare_parameter('stationary_detect_threshold_velocity', 0.1).value

        self.detect_buf = {}
        self.tf2_buffer = tf2_ros.Buffer(node=self)
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self)

    @abstractmethod
    def detected_boxes_cb(self, detected_boxes_msg):
        pass

    def preprocess_msg(self, now, detected_boxes_msg):
        # To ignore cameras which stop by accidents, remove detecion results for cameras that are not updated longer than threshold to remove track
        delete_camera_ids = []
        for key in self.detect_buf:
            if (now - rclpy.time.Time.from_msg(self.detect_buf[key].header.stamp)) > self.tracker.duration_inactive_to_remove:
                delete_camera_ids.append(key)
        for key in delete_camera_ids:
            self.get_logger().info("delete buffer for the camera which is not updated, camera ID = " + str(key))
            del self.detect_buf[key]

        self.detect_buf[detected_boxes_msg.camera_id] = detected_boxes_msg

        combined_msg = None
        # spatial hash buffers to detect depulicate results that are closer than IOU circle size from different camera messages
        grid_size = self.iou_circle_size
        combined_tracked_boxes_grid = defaultdict(set)  # key: (floor(x / grid_size), floor(y / grid_size)), value: {box_index}
        combined_tracked_boxes_coords = {}  # key: box_index, value: np.array([x, y])
        combined_tracked_boxes_stamps = {}  # key: box_index, value: rclpy.time.Time
        duplicate_combined_tracked_boxes_indices = set()
        for key in self.detect_buf:
            msg = copy.deepcopy(self.detect_buf[key])
            msg_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

            # detect duplicate results from different camera messages
            if combined_msg is not None:
                for idx_bbox, bbox in enumerate(msg.tracked_boxes):
                    # check all surrounding grids to avoid digitization errors
                    grid_bbox_x = math.floor(bbox.center3d.x / grid_size)
                    grid_bbox_y = math.floor(bbox.center3d.y / grid_size)
                    for grid_dx, grid_dy in itertools.product((-1, 0, 1), repeat=2):
                        grid_key = (grid_bbox_x + grid_dx, grid_bbox_y + grid_dy)
                        if grid_key in combined_tracked_boxes_grid:
                            close_bbox_indices = combined_tracked_boxes_grid[grid_key]
                            bbox_coord = np.array([bbox.center3d.x, bbox.center3d.y])

                            duplicate_bbox_dist = None
                            duplicate_bbox_stamp = None
                            duplicate_bbox_index = None
                            for close_bbox_index in close_bbox_indices:
                                close_bbox_dist = np.linalg.norm(combined_tracked_boxes_coords[close_bbox_index] - bbox_coord)
                                if (close_bbox_dist < self.iou_circle_size) and ((duplicate_bbox_dist is None) or (close_bbox_dist < duplicate_bbox_dist)):
                                    duplicate_bbox_dist = close_bbox_dist
                                    duplicate_bbox_stamp = combined_tracked_boxes_stamps[close_bbox_index]
                                    duplicate_bbox_index = close_bbox_index

                            if (duplicate_bbox_dist is not None) and (duplicate_bbox_stamp is not None) and (duplicate_bbox_index is not None):
                                # if duplicate results ard found, remove older bbox
                                if duplicate_bbox_stamp < msg_stamp:
                                    duplicate_combined_tracked_boxes_indices.add(duplicate_bbox_index)
                                else:
                                    duplicate_combined_tracked_boxes_indices.add(len(combined_msg.tracked_boxes) + idx_bbox)

            # update spatial hash buffers to detect duplicate results from combined message
            for idx_bbox, bbox in enumerate(msg.tracked_boxes):
                grid_key = (math.floor(bbox.center3d.x / grid_size), math.floor(bbox.center3d.y / grid_size))
                if combined_msg is None:
                    combined_msg_idx_bbox = idx_bbox
                else:
                    combined_msg_idx_bbox = len(combined_msg.tracked_boxes) + idx_bbox

                combined_tracked_boxes_grid[grid_key].add(combined_msg_idx_bbox)
                combined_tracked_boxes_coords[combined_msg_idx_bbox] = np.array([bbox.center3d.x, bbox.center3d.y])
                combined_tracked_boxes_stamps[combined_msg_idx_bbox] = msg_stamp

            # update combined message
            if combined_msg is None:
                combined_msg = msg
            else:
                combined_msg.tracked_boxes.extend(msg.tracked_boxes)
        combined_msg.header.stamp = now.to_msg()

        # remove depulicate results from combined message
        for duplicate_combined_tracked_box_index in sorted(duplicate_combined_tracked_boxes_indices, reverse=True):
            del combined_msg.tracked_boxes[duplicate_combined_tracked_box_index]

        detect_results = []
        center_bird_eye_global_list = []
        for idx_bbox, bbox in enumerate(combined_msg.tracked_boxes):
            detect_results.append([bbox.box.xmin, bbox.box.ymin, bbox.box.xmax, bbox.box.ymax])
            center_bird_eye_global_list.append([bbox.center3d.x, bbox.center3d.y, bbox.center3d.z])

        return combined_msg, np.array(detect_results), center_bird_eye_global_list

    def pub_result(self, msg, track_pos_dict, track_vel_dict, alive_track_id_list, stationary_track_id_list):
        transform = None
        if msg.header.frame_id != self.output_frame:
            try:
                transform = self.tf2_buffer.lookup_transform(self.output_frame, msg.header.frame_id, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=self.get_clock().clock_type),
                                                             Duration(seconds=1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().error(F"lookup_transform error from {msg.header.frame_id} to {self.output_frame}")
                return

        # init People message
        people_msg = People()
        people_msg.header = copy.deepcopy(msg.header)
        people_msg.header.frame_id = self.output_frame

        for track_id in track_pos_dict.keys():
            # create Person message
            person = Person()
            person.name = str(track_id)
            if msg.header.frame_id != self.output_frame:
                if transform is None:
                    self.get_logger().error("transform is not available")
                    return
                pos_stamped = PointStamped()
                pos_stamped.header = msg.header
                pos_stamped.point.x = track_pos_dict[track_id][0]
                pos_stamped.point.y = track_pos_dict[track_id][1]
                pos_stamped.point.z = 0.0

                vel_stamped = Vector3Stamped()
                vel_stamped.header = msg.header
                vel_stamped.vector.x = track_vel_dict[track_id][0]
                vel_stamped.vector.y = track_vel_dict[track_id][1]
                vel_stamped.vector.z = 0.0

                try:
                    transform_pos_stamped = do_transform_point(pos_stamped, transform)
                    transform_vel_stamped = do_transform_vector3(vel_stamped, transform)

                    person.position = transform_pos_stamped.point
                    person.velocity.x = transform_vel_stamped.vector.x
                    person.velocity.y = transform_vel_stamped.vector.y
                    person.velocity.z = 0.0
                except RuntimeError as e:
                    self.get_logger().error(F"transform error, {e}")
                    return
            else:
                person.position.x = track_pos_dict[track_id][0]
                person.position.y = track_pos_dict[track_id][1]
                person.position.z = 0.0
                person.velocity.x = track_vel_dict[track_id][0]
                person.velocity.y = track_vel_dict[track_id][1]
                person.velocity.z = 0.0
            if track_id in alive_track_id_list:
                person.reliability = 1.0
            else:
                person.reliability = 0.9

            # check if track is in stationary state
            if track_id in stationary_track_id_list:
                person.tags.append("stationary")

            people_msg.people.append(person)

        self.people_pub.publish(people_msg)

        return people_msg

    def vis_result(self, people_msg, alive_track_id_list, stationary_track_id_list):
        # publish visualization marker array for rviz
        marker_array = MarkerArray()
        # plot sphere for current position, arrow for current direction
        for person in people_msg.people:
            track_id = int(person.name)

            track_color = self.list_colors[track_id % self.n_colors]

            marker = Marker()
            marker.header = people_msg.header
            marker.ns = "track-origin-position"
            marker.id = track_id*2
            if track_id in stationary_track_id_list:
                marker.type = Marker.CUBE
            else:
                marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = Duration(nanoseconds=1e8).to_msg()
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.1
            marker.pose.position.x = person.position.x
            marker.pose.position.y = person.position.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.r = track_color[0]
            marker.color.g = track_color[1]
            marker.color.b = track_color[2]
            if track_id in alive_track_id_list:
                marker.color.a = 1.0
            else:
                marker.color.a = 0.5
            marker_array.markers.append(marker)

            velocity_norm = np.linalg.norm([person.velocity.x, person.velocity.y])
            velocity_orientation = math.atan2(person.velocity.y, person.velocity.x)
            velocity_orientation_quat = quaternion_from_euler(0.0, 0.0, velocity_orientation)
            marker = Marker()
            marker.header = people_msg.header
            marker.ns = "track-origin-direction"
            marker.id = track_id*2+1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = Duration(nanoseconds=1e8).to_msg()
            marker.scale.x = 1.0 * min(velocity_norm, 1.0)
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.pose.position.x = person.position.x
            marker.pose.position.y = person.position.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = velocity_orientation_quat[0]
            marker.pose.orientation.y = velocity_orientation_quat[1]
            marker.pose.orientation.z = velocity_orientation_quat[2]
            marker.pose.orientation.w = velocity_orientation_quat[3]
            marker.color.r = track_color[0]
            marker.color.g = track_color[1]
            marker.color.b = track_color[2]
            if track_id in alive_track_id_list:
                marker.color.a = 1.0
            else:
                marker.color.a = 0.5
            marker_array.markers.append(marker)

        self.vis_marker_array_pub.publish(marker_array)
