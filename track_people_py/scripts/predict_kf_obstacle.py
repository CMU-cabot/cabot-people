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

import math
import signal
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from people_msgs.msg import People, Person
from matplotlib import pyplot as plt

from track_people_py import PredictKfAbstract


class PredictKfObstacle(PredictKfAbstract):
    def __init__(self):
        super().__init__('predict_obstacle_py')

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
                if track_id not in self.predict_buf.track_id_stationary_start_time_dict:
                    # record time to start stationary state
                    self.predict_buf.track_id_stationary_start_time_dict[track_id] = rclpy.time.Time.from_msg(msg.header.stamp)
                else:
                    # add stationary tag if enough time passes after starting stationary state
                    stationary_start_time = self.predict_buf.track_id_stationary_start_time_dict[track_id]
                    if (rclpy.time.Time.from_msg(msg.header.stamp) - stationary_start_time).nanoseconds/1e9 > self.stationary_detect_threshold_duration_:
                        person.tags.append("stationary")
            elif track_id in self.predict_buf.track_id_stationary_start_time_dict:
                # clear time to start stationary state
                del self.predict_buf.track_id_stationary_start_time_dict[track_id]

            people_msg.people.append(person)

        self.people_pub.publish(people_msg)


def main():
    rclpy.init()

    predict_people = PredictKfObstacle()

    plt.ion()
    plt.show()
    rclpy.spin(predict_people)


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)

if __name__ == '__main__':
    main()
