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

from matplotlib import pyplot as plt
import numpy as np
import rclpy
import rclpy.node
from rclpy.duration import Duration
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_updater import Updater
from diagnostic_updater import HeaderlessTopicDiagnostic
from diagnostic_updater import FrequencyStatusParam

from track_people_msgs.msg import TrackedBox
from track_people_msgs.msg import TrackedBoxes


class AbsTrackPeople(rclpy.node.Node):
    __metaclass__ = ABCMeta

    def __init__(self):
        super().__init__('track_people_py')

        self.iou_threshold = self.declare_parameter('iou_threshold', 0.01).value
        self.iou_circle_size = self.declare_parameter('iou_circle_size', 0.5).value
        self.kf_init_var = self.declare_parameter('kf_init_var', 1.0).value
        self.kf_process_var = self.declare_parameter('kf_process_var', 1.0).value
        self.kf_measure_var = self.declare_parameter('kf_measure_var', 1.0).value
        self.minimum_valid_track_duration = self.declare_parameter('minimum_valid_track_duration', 0.0).value
        self.duration_inactive_to_remove = self.declare_parameter('duration_inactive_to_remove', 2.0).value

        self.detected_boxes_sub = self.create_subscription(TrackedBoxes, 'people/detected_boxes', self.detected_boxes_cb, 10)
        self.tracked_boxes_pub = self.create_publisher(TrackedBoxes, 'people/tracked_boxes', 10)
        self.visualization_marker_array_pub = self.create_publisher(MarkerArray, 'people/tracking_visualization', 10)

        self.frame_id = 0
        self.prev_detect_time_sec = 0

        self.updater = Updater(self)

        target_fps = self.declare_parameter('target_fps', 0.0).value
        diagnostic_name = self.declare_parameter('diagnostic_name', "PeopleTrack").value
        self.htd = HeaderlessTopicDiagnostic(diagnostic_name, self.updater,
                                             FrequencyStatusParam({'min': target_fps/3.0, 'max': target_fps}, 0.2, 2))

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

    def pub_result(self, detected_boxes_msg, id_list, color_list, tracked_duration):
        # publish tracked boxes message
        tracked_boxes_msg = TrackedBoxes()
        tracked_boxes_msg.header = detected_boxes_msg.header
        tracked_boxes_msg.camera_id = detected_boxes_msg.camera_id
        tracked_boxes_msg.pose = detected_boxes_msg.pose
        for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
            if tracked_duration[idx_bbox] < self.minimum_valid_track_duration:
                continue
            tracked_box = TrackedBox()
            tracked_box.header = bbox.header
            tracked_box.track_id = id_list[idx_bbox]
            tracked_box.color = ColorRGBA(r=color_list[idx_bbox][0], g=color_list[idx_bbox][1], b=color_list[idx_bbox][2], a=0.0)
            tracked_box.box = bbox.box
            tracked_box.center3d = bbox.center3d
            tracked_boxes_msg.tracked_boxes.append(tracked_box)
        self.tracked_boxes_pub.publish(tracked_boxes_msg)

        self.get_logger().info("camera ID = " + detected_boxes_msg.camera_id + ", number of tracked people = " + str(len(tracked_boxes_msg.tracked_boxes)))

    def vis_result(self, detected_boxes_msg, id_list, color_list, tracked_duration):
        # publish visualization marker array for rviz
        marker_array = MarkerArray()
        for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
            if tracked_duration[idx_bbox] < self.minimum_valid_track_duration:
                continue
            marker = Marker()
            marker.header = bbox.header
            marker.ns = "track-people"
            marker.id = id_list[idx_bbox]
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.lifetime = Duration(nanoseconds=500000000).to_msg()
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.2
            marker.pose.position = bbox.center3d
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.r = color_list[idx_bbox][0]
            marker.color.g = color_list[idx_bbox][1]
            marker.color.b = color_list[idx_bbox][2]
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.visualization_marker_array_pub.publish(marker_array)
