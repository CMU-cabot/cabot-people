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

from filterpy.common import Q_discrete_white_noise
import numpy as np
from scipy.linalg import block_diag
from scipy.optimize import linear_sum_assignment

from . import reid_utils_fn
from . import kf_utils


class TrackerSort3D:
    def __init__(self, iou_threshold=0.01, iou_circle_size=0.5, kf_init_var=1.0, kf_process_var=1.0, kf_measure_var=1.0,
                 minimum_valid_track_observe=5, duration_inactive_to_remove=2.0, duration_inactive_to_stop_publish=0.2,
                 stationary_detect_threshold_duration=1.0, stationary_detect_threshold_velocity=0.1):
        # Initialization
        #
        # iou_threshold : minimum IOU threshold to keep track
        # iou_circle_size : radius of circle in bird-eye view to calculate IOU
        # kf_init_var : variance for initial state covariance matrix
        # kf_process_var : variance for process noise covariance matrix
        # kf_measure_var : variance for measurement noise covariance matrix
        # minimum_valid_track_observe : minimum number of observations to start output tracks
        # duration_inactive_to_remove : duration for an inactive detection to be removed
        # duration_inactive_to_stop_publish : duration to stop output tracks
        # stationary_detect_threshold_duration : threshold duration to detect as stationary
        # stationary_detect_threshold_velocity : threshold velocity to detect as staionary

        # parameters for Kalman Filter
        self.kf_init_time_step = 1.0  # initial time step, this value does not affect results
        self.kf_init_var = kf_init_var
        self.kf_process_var = kf_process_var  # process noise : smaller value will be smoother
        self.kf_measure_var = kf_measure_var  # measurement noise

        # buffers to update Kalaman Filter
        self.kf_dict = {}
        self.kf_last_input_dict = {}
        self.kf_input_count_dict = {}
        self.kf_last_predict_time_dict = {}
        self.kf_expire_time_dict = {}
        self.kf_since_time_dict = {}

        # buffers to check track status
        self.track_missing_time_dict = {}
        self.track_vel_hist_dict = {}
        self.track_stationary_start_time_dict = {}

        # set parameters
        self.iou_threshold = iou_threshold
        self.iou_circle_size = iou_circle_size
        self.minimum_valid_track_observe = minimum_valid_track_observe
        self.duration_inactive_to_remove = duration_inactive_to_remove
        self.duration_inactive_to_stop_publish = duration_inactive_to_stop_publish
        self.stationary_detect_threshold_duration = stationary_detect_threshold_duration
        self.stationary_detect_threshold_velocity = stationary_detect_threshold_velocity

        # counter of tracks
        self.tracker_count = 0

    def _predict_kf(self, id_track, now):
        # set time steps
        dt = (now - self.kf_last_predict_time_dict[id_track]).nanoseconds/1000000000
        self.kf_dict[id_track].F = np.array([[1, dt, 0,  0],
                                             [0,  1, 0,  0],
                                             [0,  0, 1, dt],
                                             [0,  0, 0,  1]])
        q = Q_discrete_white_noise(dim=2, dt=dt, var=self.kf_process_var)
        self.kf_dict[id_track].Q = block_diag(q, q)

        # run predict
        self.kf_dict[id_track].predict()

        # set last predict time
        self.kf_last_predict_time_dict[id_track] = now

    def _track(self, now, bboxes, center_pos_list):
        # Performs tracking by comparing with previous detected people
        #
        # INPUT
        # now : timestamp
        # bboxes : n x 4+ matrix - each row is a bbox [x_tl, y_tl, x_br, y_br]
        #                           of each detection. Additional element (e.g.
        #                           detection score) may be included.
        # center_pos_list : n x 2 matrix - each row is a bbox [x, y]
        #                           in bird-eye view of each detection.
        #
        # OUTPUT
        # prev_exist : boolean n-vector indicating whether each of the
        #               person exists before
        # person_id : int n-vector indicating id of each person
        # tracked_duration : total tracked duration

        center_circle_list = []
        for center_pos in center_pos_list:
            center_circle_list.append([center_pos[0], center_pos[1], self.iou_circle_size])

        # prepare output
        prev_exist = np.zeros(len(bboxes)).astype(np.bool8)
        person_id = np.zeros(len(bboxes)).astype(np.uint32)
        tracked_duration = np.zeros(len(bboxes)).astype(np.float32)
        if (len(bboxes) == 0) and (len(self.kf_dict) == 0):
            # No new detection and no active tracks
            # Do nothing
            det_to_add = []
            track_inactive = []

            # predict box by Kalman Filter
            for id_track in self.kf_dict.keys():
                self._predict_kf(id_track, now)
        elif (len(bboxes) == 0) and (len(self.kf_dict) > 0):
            # No new detection but has active tracks

            # no tracks to add
            det_to_add = []

            # set all active tracks to inactive
            track_inactive = list(self.kf_dict.keys())

            # predict box by Kalman Filter
            for id_track in self.kf_dict.keys():
                self._predict_kf(id_track, now)
        elif (len(bboxes) > 0) and (len(self.kf_dict) == 0):

            # If no active detection, add all of them
            det_to_add = np.arange(len(bboxes))

            # No track to be considered inactive
            track_inactive = []

            # predict box by Kalman Filter
            for id_track in self.kf_dict.keys():
                self._predict_kf(id_track, now)
        elif (len(bboxes) > 0) and (len(self.kf_dict) > 0):
            # If there are active detections, compared them to new detections
            # then decide to match or add as new tracks

            # predict circle by Kalman Filter
            kf_pred_circles = []
            for id_track in self.kf_dict.keys():
                self._predict_kf(id_track, now)

                kf_x = self.kf_dict[id_track].x[0, 0]
                kf_y = self.kf_dict[id_track].x[2, 0]
                kf_circle = [kf_x, kf_y, self.iou_circle_size]
                kf_pred_circles.append(kf_circle)

            # get all active tracks
            key_kf = list(self.kf_dict.keys())

            # compute IOU
            iou = reid_utils_fn.compute_circle_pairwise_iou(center_circle_list, kf_pred_circles)

            # match by Hungarian
            row_ind, col_ind = linear_sum_assignment(1-iou)

            # get prev and current detection correspondence, then update existing tracks
            track_continue_current = []
            track_continue_prev = []
            for cur_idx, prev_idx in zip(row_ind, col_ind):
                if iou[cur_idx][prev_idx] > self.iou_threshold:
                    track_continue_current.append(cur_idx)
                    track_continue_prev.append(prev_idx)
            track_continue_prev = [key_kf[i] for i in track_continue_prev]  # get id of still active tracks

            # Now we have a 1-1 correspondence of active tracks
            # between track id in track_continue_prev and detection in track_continue_current.
            # Add these infos to the record.
            for i, id_track in enumerate(track_continue_prev):
                circle_tmp = center_circle_list[track_continue_current[i]]
                self.kf_expire_time_dict[id_track] = now + self.duration_inactive_to_remove

                # update Kalman Filter
                kf_meas = [circle_tmp[0], circle_tmp[1]]
                self.kf_dict[id_track].update(np.asarray(kf_meas).reshape([len(kf_meas), 1]))

                # set output
                prev_exist[track_continue_current[i]] = True
                person_id[track_continue_current[i]] = id_track
                tracked_duration[track_continue_current[i]] = (now - self.kf_since_time_dict[id_track]).nanoseconds/1000000000

            # the rest of the tracks are new tracks to be add later
            det_to_add = np.setdiff1d(np.arange(len(bboxes)), track_continue_current)

            # get the list of tracks that have become inactive
            track_inactive = [x for x in key_kf if x not in track_continue_prev]
        else:
            assert False, "Something is wrong here... All conditions shouldn't be wrong!"

        # deal with inactive tracks
        for id_track in track_inactive:
            if now > self.kf_expire_time_dict[id_track]:
                # remove tracks that have been inactive for too long
                del self.kf_dict[id_track]
                del self.kf_last_input_dict[id_track]
                del self.kf_input_count_dict[id_track]
                del self.kf_last_predict_time_dict[id_track]
                del self.kf_expire_time_dict[id_track]
                del self.kf_since_time_dict[id_track]

        # add new trackers
        for id_track in det_to_add:
            self.kf_expire_time_dict[self.tracker_count] = now + self.duration_inactive_to_remove
            self.kf_since_time_dict[self.tracker_count] = now
            self.kf_last_predict_time_dict[self.tracker_count] = now

            # save active Kalaman Filter
            new_kf_x = center_circle_list[id_track][0]
            new_kf_y = center_circle_list[id_track][1]
            self.kf_dict[self.tracker_count] = kf_utils.init_kf_fixed_size([new_kf_x, 0.0, new_kf_y, 0.0],
                                                                           self.kf_init_time_step, self.kf_init_var, self.kf_process_var, self.kf_measure_var)

            # set output
            prev_exist[id_track] = False
            person_id[id_track] = self.tracker_count
            tracked_duration[id_track] = (now - self.kf_since_time_dict[self.tracker_count]).nanoseconds/1000000000

            self.tracker_count += 1

        return prev_exist, [int(x) for x in person_id], tracked_duration

    def track(self, now, bboxes, center_pos_list):
        # Performs tracking and output valid results
        #
        # INPUT
        # now : timestamp
        # bboxes : n x 4+ matrix - each row is a bbox [x_tl, y_tl, x_br, y_br]
        #                           of each detection. Additional element (e.g.
        #                           detection score) may be included.
        # center_pos_list : n x 2 matrix - each row is a bbox [x, y]
        #                           in bird-eye view of each detection.
        #
        # OUTPUT
        # track_pos_dict : dictionary of track id and positions in bird eye view coordinate
        # track_vel_dict : dictionary of track id and velocities in bird eye view coordinate
        # alive_track_id_list : list of track id which are visible now
        # stationary_track_id_list : list of track id which are stationary

        _, alive_track_id_list, _ = self._track(now, bboxes, center_pos_list)

        # update track input
        for (track_id, center3d) in zip(alive_track_id_list, center_pos_list):
            self.kf_last_input_dict[track_id] = center3d
            if track_id not in self.kf_input_count_dict:
                self.kf_input_count_dict[track_id] = 1
            else:
                self.kf_input_count_dict[track_id] += 1

            # clear missing time
            if track_id in self.track_missing_time_dict:
                del self.track_missing_time_dict[track_id]

        track_pos_dict = {}
        track_vel_dict = {}
        # add position and velocity of visible tracks
        for track_id in alive_track_id_list:
            if (track_id not in self.kf_input_count_dict) or (self.kf_input_count_dict[track_id] < self.minimum_valid_track_observe):
                continue
            # save position and velocity
            # use raw position
            track_pos_dict[track_id] = self.kf_last_input_dict[track_id][:2]
            # ues filtered position
            # track_pos_dict[track_id] = self.kf_dict[track_id].x.reshape(1, 4)[0, [0, 2]]
            track_vel_dict[track_id] = self.kf_dict[track_id].x.reshape(1, 4)[0, [1, 3]]

            # update buffers to check track status
            if track_id not in self.track_vel_hist_dict:
                self.track_vel_hist_dict[track_id] = deque(maxlen=self.minimum_valid_track_observe)
            self.track_vel_hist_dict[track_id].append(track_vel_dict[track_id])

        # clean up missed track if necessary
        missing_track_id_list = set(self.kf_last_input_dict.keys()) - set(alive_track_id_list)
        stop_publish_track_id_list = set()
        for track_id in missing_track_id_list:
            # update missing time
            if track_id not in self.track_missing_time_dict:
                self.track_missing_time_dict[track_id] = now

            # if missing long time, stop publishing in people topic
            if (now - self.track_missing_time_dict[track_id]).nanoseconds/1000000000 > self.duration_inactive_to_stop_publish:
                stop_publish_track_id_list.add(track_id)

            # if Kalman Filter is deleteted, delete buffers to check track status
            if track_id not in self.kf_dict:
                if track_id in self.track_vel_hist_dict:
                    del self.track_vel_hist_dict[track_id]
                del self.track_missing_time_dict[track_id]
                if track_id in self.track_stationary_start_time_dict:
                    del self.track_stationary_start_time_dict[track_id]

        # add position and velocity of undeleted missing tracks
        publish_missing_track_id_list = missing_track_id_list - stop_publish_track_id_list
        for track_id in publish_missing_track_id_list:
            if (track_id not in self.kf_input_count_dict) or (self.kf_input_count_dict[track_id] < self.minimum_valid_track_observe):
                continue
            # save position and velocity
            # use raw position
            track_pos_dict[track_id] = self.kf_last_input_dict[track_id][:2]
            # ues filtered position
            # track_pos_dict[track_id] = self.kf_dict[track_id].x.reshape(1, 4)[0, [0, 2]]
            track_vel_dict[track_id] = self.kf_dict[track_id].x.reshape(1, 4)[0, [1, 3]]

        # update track stationary start time, and stationary track list
        stationary_track_id_list = []
        for track_id in track_pos_dict.keys():
            # calculate median velocity norm of track in recent time window to remove noise
            track_vel_norm_hist = [np.linalg.norm(v) for v in self.track_vel_hist_dict[track_id]]
            track_vel_norm_median = np.median(track_vel_norm_hist)

            # check if track is in stationary state
            if track_vel_norm_median < self.stationary_detect_threshold_velocity:
                if track_id not in self.track_stationary_start_time_dict:
                    # record time to start stationary state
                    self.track_stationary_start_time_dict[track_id] = now
                else:
                    # add stationary list if enough time passes after starting stationary state
                    stationary_start_time = self.track_stationary_start_time_dict[track_id]
                    if (now - stationary_start_time).nanoseconds/1e9 > self.stationary_detect_threshold_duration:
                        stationary_track_id_list.append(track_id)
            elif track_id in self.track_stationary_start_time_dict:
                # clear time to start stationary state
                del self.track_stationary_start_time_dict[track_id]

        return track_pos_dict, track_vel_dict, alive_track_id_list, stationary_track_id_list
