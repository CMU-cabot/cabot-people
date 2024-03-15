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

import cv2
from mmdeploy_runtime import Detector
import numpy as np
import rclpy

from track_people_py import AbsDetectPeople


class DetectMMDetSegPeople(AbsDetectPeople):
    __metaclass__ = ABCMeta

    def __init__(self, device):
        super().__init__(device)

        # load detect model
        self.model_input_width = self.declare_parameter('model_input_width', 416).value
        self.model_input_height = self.declare_parameter('model_input_height', 416).value
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
        resize_rgb_img = cv2.resize(rgb_img, (self.model_input_width, self.model_input_height))
        bboxes, labels, masks = self.detector(resize_rgb_img)
        return (labels, bboxes[:,4], bboxes[:,0:4], masks)

    def post_process(self, rgb_img, frame_resized, boxes_res):
        resize_width_ratio = self.model_input_width / float(rgb_img.shape[1])
        resize_height_ratio = self.model_input_height / float(rgb_img.shape[0])

        people_res = []
        mask_results = []
        for idx, score, box, mask in zip(*boxes_res):
            if (idx==0) and (score>self.detection_threshold):
                # convert results to format [xtl, ytl, xbr, ybr, conf, class]
                # 0 is class ID of 'person' class
                xtl = box[0]
                ytl = box[1]
                xbr = box[2]
                ybr = box[3]

                # resize detected box to original image size
                xtl = int(xtl / resize_width_ratio)
                ytl = int(ytl / resize_height_ratio)
                xbr = int(xbr / resize_width_ratio)
                ybr = int(ybr / resize_height_ratio)
                people_res.append([xtl, ytl, xbr, ybr, score, 1])

                # resize mask to original image size
                mask = cv2.resize(mask, (rgb_img.shape[1], rgb_img.shape[0]))
                mask_results.append(mask.tolist())
        detect_results = np.array(people_res)
        mask_results = np.array(mask_results, dtype=np.uint8)

        if len(detect_results) > 0:
            # delete small detections
            small_detection = np.where(detect_results[:, 3]-detect_results[:, 1] < self.minimum_detection_size_threshold)[0]
            detect_results = np.delete(detect_results, small_detection, axis=0)
            mask_results = np.delete(mask_results, small_detection, axis=0)

        return detect_results, mask_results


def main():
    rclpy.init()
    device = "cuda"

    detect_people = DetectMMDetSegPeople(device)

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
