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
import math

from diagnostic_updater import Updater
from diagnostic_updater import HeaderlessTopicDiagnostic
from diagnostic_updater import FrequencyStatusParam
import numpy as np
import rclpy
import rclpy.node
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from people_msgs.msg import People, Person
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from track_people_msgs.msg import TrackedBoxes


class TrackBuffer():
    def __init__(self):
        self.track_input_queue_dict = {}
        self.track_time_queue_dict = {}
        self.track_vel_hist_dict = {}
        self.track_color_dict = {}

        self.track_id_missing_time_dict = {}
        self.track_id_stationary_start_time_dict = {}


class AbsTrackPeople(rclpy.node.Node):
    __metaclass__ = ABCMeta

    def __init__(self):
        super().__init__('track_people_py')

        self.iou_threshold = self.declare_parameter('iou_threshold', 0.01).value
        self.iou_circle_size = self.declare_parameter('iou_circle_size', 0.5).value
        self.kf_init_var = self.declare_parameter('kf_init_var', 1.0).value
        self.kf_process_var = self.declare_parameter('kf_process_var', 1.0).value
        self.kf_measure_var = self.declare_parameter('kf_measure_var', 1.0).value
        # number of frames to start publish people topic
        self.input_time = self.declare_parameter('input_time', 5).value
        self.minimum_valid_track_duration = self.declare_parameter('minimum_valid_track_duration', 0.0).value
        # duration (seconds) to remove an inactive track
        self.duration_inactive_to_remove = self.declare_parameter('duration_inactive_to_remove', 2.0).value
        # duration (seconds) to stop publishing an inactive track in people topic
        self.duration_inactive_to_stop_publish = self.declare_parameter('duration_inactive_to_stop_publish', 0.2).value
        # tracked people buffer
        self.track_buf = TrackBuffer()

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

    @abstractmethod
    def detected_boxes_cb(self, detected_boxes_msg):
        pass

    def preprocess_msg(self, detected_boxes_msg):
        detect_results = []
        center_bird_eye_global_list = []
        for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
            detect_results.append([bbox.box.xmin, bbox.box.ymin, bbox.box.xmax, bbox.box.ymax])
            center_bird_eye_global_list.append([bbox.center3d.x, bbox.center3d.y, bbox.center3d.z])
        return np.array(detect_results), center_bird_eye_global_list

    def pub_result(self, msg, alive_track_id_list, track_pos_dict, track_vel_dict, track_vel_hist_dict):
        # init People message
        people_msg = People()
        people_msg.header = msg.header

        for track_id in track_pos_dict.keys():
            past_center3d = Point()
            past_center3d.x = track_pos_dict[track_id][0]
            past_center3d.y = track_pos_dict[track_id][1]
            past_center3d.z = 0.0

            # create Person message
            person = Person()
            person.name = str(track_id)
            person.position = past_center3d
            person.velocity = Point()
            person.velocity.x = track_vel_dict[track_id][0]
            person.velocity.y = track_vel_dict[track_id][1]
            person.velocity.z = 0.0
            if track_id in alive_track_id_list:
                person.reliability = 1.0
            else:
                person.reliability = 0.9

            # calculate median velocity of track in recent time window to remove noise
            track_vel_hist = []
            for (track_hist_time, track_hist_vel) in track_vel_hist_dict[track_id]:
                track_vel_hist.append(np.linalg.norm([track_hist_vel[0], track_hist_vel[1]]))
            track_vel_hist_median = np.median(track_vel_hist)

            # check if track is in stationary state
            if track_vel_hist_median < self.stationary_detect_threshold_velocity_:
                if track_id not in self.track_buf.track_id_stationary_start_time_dict:
                    # record time to start stationary state
                    self.track_buf.track_id_stationary_start_time_dict[track_id] = rclpy.time.Time.from_msg(msg.header.stamp)
                else:
                    # add stationary tag if enough time passes after starting stationary state
                    stationary_start_time = self.track_buf.track_id_stationary_start_time_dict[track_id]
                    if (rclpy.time.Time.from_msg(msg.header.stamp) - stationary_start_time).nanoseconds/1e9 > self.stationary_detect_threshold_duration_:
                        person.tags.append("stationary")
            elif track_id in self.track_buf.track_id_stationary_start_time_dict:
                # clear time to start stationary state
                del self.track_buf.track_id_stationary_start_time_dict[track_id]

            people_msg.people.append(person)

        self.people_pub.publish(people_msg)

    def vis_result(self, msg, alive_track_id_list, track_pos_dict, track_vel_dict):
        # publish visualization marker array for rviz
        marker_array = MarkerArray()
        # plot sphere for current position, arrow for current direction
        for track_id in track_pos_dict.keys():
            marker = Marker()
            marker.header = msg.header
            marker.ns = "track-origin-position"
            marker.id = track_id*2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = Duration(nanoseconds=1e8).to_msg()
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.1
            marker.pose.position.x = track_pos_dict[track_id][0]
            marker.pose.position.y = track_pos_dict[track_id][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.r = self.track_buf.track_color_dict[track_id][0]
            marker.color.g = self.track_buf.track_color_dict[track_id][1]
            marker.color.b = self.track_buf.track_color_dict[track_id][2]
            if track_id in alive_track_id_list:
                marker.color.a = 1.0
            else:
                marker.color.a = 0.5
            marker_array.markers.append(marker)

            track_velocity = track_vel_dict[track_id]
            velocity_norm = np.linalg.norm(np.array(track_velocity))
            velocity_orientation = math.atan2(track_velocity[1], track_velocity[0])
            velocity_orientation_quat = quaternion_from_euler(0.0, 0.0, velocity_orientation)
            marker = Marker()
            marker.header = msg.header
            marker.ns = "track-origin-direction"
            marker.id = track_id*2+1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = Duration(nanoseconds=1e8).to_msg()
            marker.scale.x = 1.0 * min(velocity_norm, 1.0)
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.pose.position.x = track_pos_dict[track_id][0]
            marker.pose.position.y = track_pos_dict[track_id][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = velocity_orientation_quat[0]
            marker.pose.orientation.y = velocity_orientation_quat[1]
            marker.pose.orientation.z = velocity_orientation_quat[2]
            marker.pose.orientation.w = velocity_orientation_quat[3]
            marker.color.r = self.track_buf.track_color_dict[track_id][0]
            marker.color.g = self.track_buf.track_color_dict[track_id][1]
            marker.color.b = self.track_buf.track_color_dict[track_id][2]
            if track_id in alive_track_id_list:
                marker.color.a = 1.0
            else:
                marker.color.a = 0.5
            marker_array.markers.append(marker)

        self.vis_marker_array_pub.publish(marker_array)
