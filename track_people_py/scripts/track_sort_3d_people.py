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

from collections import deque
import copy
import signal
import sys
import traceback

import numpy as np
import rclpy
from rclpy.duration import Duration

from track_people_py import AbsTrackPeople
from track_people_py.track_utils import TrackerSort3D


class TrackSort3dPeople(AbsTrackPeople):
    def __init__(self):
        super().__init__()

        # set tracker
        self.tracker = TrackerSort3D(iou_threshold=self.iou_threshold, iou_circle_size=self.iou_circle_size,
                                     kf_init_var=self.kf_init_var, kf_process_var=self.kf_process_var, kf_measure_var=self.kf_measure_var,
                                     minimum_valid_track_duration=Duration(seconds=self.minimum_valid_track_duration),
                                     duration_inactive_to_remove=Duration(seconds=self.duration_inactive_to_remove))

        self.detect_buf = {}

    def update_track_buf(self, now, alive_track_id_list, center_bird_eye_global_list):
        # update queue
        for (track_id, center3d) in zip(alive_track_id_list, center_bird_eye_global_list):
            # update tracked people buffer
            if track_id not in self.track_buf.track_input_queue_dict:
                self.track_buf.track_input_queue_dict[track_id] = deque(maxlen=self.input_time)
            self.track_buf.track_input_queue_dict[track_id].append(center3d)

            # clear missing time
            if track_id in self.track_buf.track_id_missing_time_dict:
                del self.track_buf.track_id_missing_time_dict[track_id]

        track_pos_dict = {}
        track_vel_dict = {}
        for track_id in alive_track_id_list:
            if len(self.track_buf.track_input_queue_dict[track_id]) < self.input_time:
                continue
            # save position and velocity
            # use raw position
            track_pos_dict[track_id] = np.array(self.track_buf.track_input_queue_dict[track_id])[-1, :2]
            # ues filtered position
            # track_pos_dict[track_id] = self.tracker.kf_active[track_id].x.reshape(1, 4)[0, [0, 2]]
            track_vel_dict[track_id] = self.tracker.kf_active[track_id].x.reshape(1, 4)[0, [1, 3]]
            if track_id not in self.track_buf.track_vel_hist_dict:
                self.track_buf.track_vel_hist_dict[track_id] = deque(maxlen=self.input_time)
            self.track_buf.track_vel_hist_dict[track_id].append(track_vel_dict[track_id])

        # clean up missed track if necessary
        missing_track_id_list = set(self.track_buf.track_input_queue_dict.keys()) - set(alive_track_id_list)
        stop_publish_track_id_list = set()
        for track_id in missing_track_id_list:
            # update missing time
            if track_id not in self.track_buf.track_id_missing_time_dict:
                self.track_buf.track_id_missing_time_dict[track_id] = now

            # if missing long time, stop publishing in people topic
            if (now - self.track_buf.track_id_missing_time_dict[track_id]).nanoseconds/1000000000 > self.duration_inactive_to_stop_publish:
                stop_publish_track_id_list.add(track_id)

            # if missing long time, delete track
            if track_id not in self.tracker.kf_active:
                del self.track_buf.track_input_queue_dict[track_id]
                if track_id in self.track_buf.track_vel_hist_dict:
                    del self.track_buf.track_vel_hist_dict[track_id]
                del self.track_buf.track_id_missing_time_dict[track_id]
                if track_id in self.track_buf.track_id_stationary_start_time_dict:
                    del self.track_buf.track_id_stationary_start_time_dict[track_id]

        # add track which is missing, but not deleted yet
        publish_missing_track_id_list = missing_track_id_list - stop_publish_track_id_list
        for track_id in publish_missing_track_id_list:
            if (track_id not in self.track_buf.track_input_queue_dict) or (len(self.track_buf.track_input_queue_dict[track_id]) < self.input_time):
                continue
            # save position and velocity
            # use raw position
            track_pos_dict[track_id] = np.array(self.track_buf.track_input_queue_dict[track_id])[-1, :2]
            # ues filtered position
            # track_pos_dict[track_id] = self.tracker.kf_active[track_id].x.reshape(1, 4)[0, [0, 2]]
            track_vel_dict[track_id] = self.tracker.kf_active[track_id].x.reshape(1, 4)[0, [1, 3]]

        # update track stationary start time, and stationary track list
        stationary_track_id_list = []
        for track_id in track_pos_dict.keys():
            # calculate median velocity norm of track in recent time window to remove noise
            track_vel_norm_hist = [np.linalg.norm(v) for v in self.track_buf.track_vel_hist_dict[track_id]]
            track_vel_norm_median = np.median(track_vel_norm_hist)

            # check if track is in stationary state
            if track_vel_norm_median < self.stationary_detect_threshold_velocity_:
                if track_id not in self.track_buf.track_id_stationary_start_time_dict:
                    # record time to start stationary state
                    self.track_buf.track_id_stationary_start_time_dict[track_id] = now
                else:
                    # add stationary list if enough time passes after starting stationary state
                    stationary_start_time = self.track_buf.track_id_stationary_start_time_dict[track_id]
                    if (now - stationary_start_time).nanoseconds/1e9 > self.stationary_detect_threshold_duration_:
                        stationary_track_id_list.append(track_id)
            elif track_id in self.track_buf.track_id_stationary_start_time_dict:
                # clear time to start stationary state
                del self.track_buf.track_id_stationary_start_time_dict[track_id]

        return track_pos_dict, track_vel_dict, stationary_track_id_list

    def detected_boxes_cb(self, detected_boxes_msg):
        self.htd.tick()
        self.get_logger().info("detected_boxes_cb")
        # check if tracker is initialized
        if not hasattr(self, 'tracker'):
            return

        now = self.get_clock().now()

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
        for key in self.detect_buf:
            msg = copy.deepcopy(self.detect_buf[key])
            if not combined_msg:
                combined_msg = msg
            else:
                combined_msg.tracked_boxes.extend(msg.tracked_boxes)
        combined_msg.header.stamp = now.to_msg()

        detect_results, center_bird_eye_global_list = self.preprocess_msg(combined_msg)

        try:
            _, alive_track_id_list, tracked_duration = self.tracker.track(now, detect_results, center_bird_eye_global_list)
        except Exception as e:
            self.get_logger().error(F"tracking error, {e}")
            self.get_logger().error(traceback.format_exc())
            return

        track_pos_dict, track_vel_dict, stationary_track_id_list = self.update_track_buf(now, alive_track_id_list, center_bird_eye_global_list)

        self.pub_result(combined_msg, alive_track_id_list, stationary_track_id_list, track_pos_dict, track_vel_dict)

        self.vis_result(combined_msg, alive_track_id_list, track_pos_dict, track_vel_dict)


def main():
    rclpy.init()

    track_people = TrackSort3dPeople()

    try:
        rclpy.spin(track_people)
    except:  # noqa: E722
        track_people.get_logger().info("Shutting down")


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)

if __name__ == '__main__':
    main()
