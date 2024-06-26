#!/usr/bin/env python3

# Copyright (c) 2021  IBM Corporation
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

import signal
import copy
import sys

from matplotlib import pyplot as plt
import rclpy
from rclpy.duration import Duration
# import time

from track_people_py import AbsTrackPeople
from track_people_py.track_utils import TrackerSort3D
from track_people_msgs.msg import TrackedBoxes


class TrackSort3dPeople(AbsTrackPeople):
    def __init__(self):
        super().__init__()

        # set tracker
        self.tracker = TrackerSort3D(iou_threshold=self.iou_threshold, iou_circle_size=self.iou_circle_size,
                                     kf_init_var=self.kf_init_var, kf_process_var=self.kf_process_var, kf_measure_var=self.kf_measure_var,
                                     minimum_valid_track_duration=Duration(seconds=self.minimum_valid_track_duration),
                                     duration_inactive_to_remove=Duration(seconds=self.duration_inactive_to_remove))

        self.combined_detected_boxes_pub = self.create_publisher(TrackedBoxes, 'people/combined_detected_boxes', 10)

        self.buffer = {}

    def detected_boxes_cb(self, detected_boxes_msg):
        self.htd.tick()
        self.get_logger().info("detected_boxes_cb")
        # check if tracker is initialized
        if not hasattr(self, 'tracker'):
            return

        now = self.get_clock().now()

        # To ignore cameras which stop by accidents, remove detecion results for cameras that are not updated longer than threshold to remove track
        delete_camera_ids = []
        for key in self.buffer:
            if (now - rclpy.time.Time.from_msg(self.buffer[key].header.stamp)) > self.tracker.duration_inactive_to_remove:
                delete_camera_ids.append(key)
        for key in delete_camera_ids:
            self.get_logger().info("delete buffer for the camera which is not updated, camera ID = " + str(key))
            del self.buffer[key]

        # 2022.01.12: remove time check for multiple detection
        # check if image is received in correct time order
        # cur_detect_time_sec = detected_boxes_msg.header.stamp.to_sec()
        # if cur_detect_time_sec<self.prev_detect_time_sec:
        #    return

        self.buffer[detected_boxes_msg.camera_id] = detected_boxes_msg

        combined_msg = None
        for key in self.buffer:
            msg = copy.deepcopy(self.buffer[key])
            if not combined_msg:
                combined_msg = msg
            else:
                combined_msg.tracked_boxes.extend(msg.tracked_boxes)
        combined_msg.header.stamp = now.to_msg()

        detect_results, center_bird_eye_global_list = self.preprocess_msg(combined_msg)

        self.combined_detected_boxes_pub.publish(combined_msg)

        try:
            _, id_list, color_list, tracked_duration = self.tracker.track(now, detect_results, center_bird_eye_global_list, self.frame_id)
        except Exception as e:
            self.get_logger().error(F"tracking error, {e}")
            return

        self.pub_result(combined_msg, id_list, color_list, tracked_duration)

        self.vis_result(combined_msg, id_list, color_list, tracked_duration)

        self.frame_id += 1
        # self.prev_detect_time_sec = cur_detect_time_sec


def main():
    rclpy.init()

    track_people = TrackSort3dPeople()

    plt.ion()
    plt.show()
    rclpy.spin(track_people)


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)

if __name__ == '__main__':
    main()
