#!/usr/bin/env python3

# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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
import sys
from abc import ABCMeta

from mmdeploy_runtime import Detector
import numpy as np
import rclpy

from track_people_py import AbsDetectPeople


class DetectMMDetPeople(AbsDetectPeople):
    __metaclass__ = ABCMeta

    def __init__(self):
        super().__init__()

        # load detect model
        detect_model_dir = self.declare_parameter('detect_model_dir', '').value

        try:
            self.detector = Detector(model_path=detect_model_dir, device_name='cuda', device_id=0)
        except:  # noqa: E722
            self.get_logger().error("cannot load model \n{}".format(detect_model_dir))

    def is_detector_initialized(self):
        if not hasattr(self, 'detector'):
            return False
        return True

    def prepare_image(self, rgb_img):
        return (None, rgb_img)

    def detect_people(self, rgb_img, frame_resized, darknet_image):
        bboxes, labels, _ = self.detector(rgb_img)
        return (labels, bboxes[:, 4], bboxes[:, 0:4])

    def post_process(self, rgb_img, frame_resized, boxes_res):
        people_res = []
        for idx, score, box in zip(*boxes_res):
            if (idx == 0) and (score > self.detection_threshold):
                # convert results to format [xtl, ytl, xbr, ybr, conf, class]
                # 0 is class ID of 'person' class
                xtl = box[0]
                ytl = box[1]
                xbr = box[2]
                ybr = box[3]
                people_res.append([xtl, ytl, xbr, ybr, score, 1])
        detect_results = np.array(people_res)

        if len(detect_results) > 0:
            # delete small detections
            small_detection = np.where((detect_results[:, 2]-detect_results[:, 0] < self.minimum_detection_size_threshold)
                                       | (detect_results[:, 3]-detect_results[:, 1] < self.minimum_detection_size_threshold))[0]
            detect_results = np.delete(detect_results, small_detection, axis=0)

        return detect_results, None


def main():
    rclpy.init()

    detect_people = DetectMMDetPeople()

    try:
        rclpy.spin(detect_people)
    except:  # noqa: E722
        detect_people.get_logger().info("Shutting down")


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)

if __name__ == '__main__':
    main()
