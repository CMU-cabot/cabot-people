#!/usr/bin/env python3

# Copyright (c) 2025  Carnegie Mellon University, IBM Corporation, and others
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
import signal
import sys

import rclpy
import rclpy.node
import std_srvs.srv
import tf2_ros

from sensor_msgs.msg import CameraInfo


class FramosInitialize():

    def __init__(self, node, map_frame_name, camera_link_frame_name, rgb_camera_topic_name, depth_camera_topic_name, min_rgb_fps, min_depth_fps,
                max_buffer_size, wait_time_after_camera_ready, wait_time_after_tf_ready):
        self.node = node
        self.logger = node.get_logger()

        self.map_frame_name = map_frame_name
        self.camera_link_frame_name = camera_link_frame_name
        self.rgb_camera_topic_name = rgb_camera_topic_name
        self.depth_camera_topic_name = depth_camera_topic_name
        self.min_rgb_fps = min_rgb_fps
        self.min_depth_fps = min_depth_fps
        self.max_buffer_size = max_buffer_size
        self.wait_time_after_camera_ready = wait_time_after_camera_ready
        self.wait_time_after_tf_ready = wait_time_after_tf_ready

        self.is_ready = False
        self.is_reset_running = False
        self.time_tf_ready = None
        self.time_camera_ready = None

        self.rgb_times = deque(maxlen=self.max_buffer_size)
        self.depth_times = deque(maxlen=self.max_buffer_size)

        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self.node)

        self.rgb_sub = self.node.create_subscription(CameraInfo, self.rgb_camera_topic_name, self.rgb_cb, 10)
        self.depth_sub = self.node.create_subscription(CameraInfo, self.depth_camera_topic_name, self.depth_cb, 10)

        self.enable_client = self.node.create_client(std_srvs.srv.SetBool, 'enable')
        self.enable_client.wait_for_service()

        self.node.create_timer(1.0, self.check_transform)

    def call_enable_client(self, data):
        request = std_srvs.srv.SetBool.Request()
        request.data = data
        return self.enable_client.call_async(request)

    def rgb_cb(self, msg):
        now = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.time_camera_ready is None:
            self.time_camera_ready = now
        elif (now - self.time_camera_ready).nanoseconds * 1.0e-9 > self.wait_time_after_camera_ready:
            self.rgb_times.append(now)

    def depth_cb(self, msg):
        now = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.time_camera_ready is None:
            self.time_camera_ready = now
        elif (now - self.time_camera_ready).nanoseconds * 1.0e-9 > self.wait_time_after_camera_ready:
            self.depth_times.append(now)

    def reset_framos(self):
        def enable_done_callback(future_enable):
            result = future_enable.result()
            if result.success:
                self.logger.info("Successed to enable FRAMOS")
            else:
                self.logger.warn("Failed to enable FRAMOS")
            self.is_reset_running = False

        def disable_done_callback(future_disable):
            result = future_disable.result()
            if result.success:
                self.logger.info("Successed to disable FRAMOS")

                future_enable = self.call_enable_client(True)
                future_enable.add_done_callback(enable_done_callback)
            else:
                self.logger.warn("Failed to disable FRAMOS")
                self.is_reset_running = False

        self.is_reset_running = True

        self.time_camera_ready = None
        self.rgb_times = deque(maxlen=self.max_buffer_size)
        self.depth_times = deque(maxlen=self.max_buffer_size)

        future_disable = self.call_enable_client(False)
        future_disable.add_done_callback(disable_done_callback)

    def check_transform(self):
        if self.is_reset_running:
            return

        try:
            t = self.tf2_buffer.lookup_transform(self.map_frame_name, self.camera_link_frame_name, rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=self.node.get_clock().clock_type))
            if self.time_tf_ready is None:
                self.time_tf_ready = self.node.get_clock().now()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.logger.info("TF is not ready")
            self.time_tf_ready = None
            return

        now = self.node.get_clock().now()
        if (len(self.rgb_times)==self.rgb_times.maxlen) and (len(self.depth_times)==self.depth_times.maxlen):
            rgb_fps = self.rgb_times.maxlen / ((now - self.rgb_times[0]).nanoseconds * 1.0e-9)
            depth_fps = self.depth_times.maxlen / ((now - self.depth_times[0]).nanoseconds * 1.0e-9)

            if (rgb_fps >= self.min_rgb_fps) and (depth_fps >= self.min_depth_fps):
                self.logger.info("FRAMOS is ready, RGB FPS: %.2f, depth FPS: %.2f" % (rgb_fps, depth_fps))
                self.is_ready = True
            else:
                self.logger.warn("Reset FRAMOS because FPS is low, RGB FPS: %.2f, depth FPS: %.2f" % (rgb_fps, depth_fps))
                self.reset_framos()
        elif (now - self.time_tf_ready).nanoseconds * 1.0e-9 > self.wait_time_after_tf_ready:
            self.logger.warn("Reset FRAMOS because FRAMOS seems not working")
            self.reset_framos()


def main():
    rclpy.init()
    node = rclpy.create_node("framos_initialize")
    map_frame_name = node.declare_parameter('map_frame', 'map').value
    camera_link_frame_name = node.declare_parameter('camera_link_frame', 'camera_link').value
    rgb_camera_topic_name = node.declare_parameter('rgb_camera_topic_name', 'color/camera_info').value
    depth_camera_topic_name = node.declare_parameter('depth_camera_topic_name', 'aligned_depth_to_color/camera_info').value
    min_rgb_fps = node.declare_parameter('min_rgb_fps', 10.0).value
    min_depth_fps = node.declare_parameter('min_depth_fps', 10.0).value
    max_buffer_size = node.declare_parameter('max_buffer_size', 100).value
    wait_time_after_camera_ready = node.declare_parameter('wait_time_after_camera_ready', 10.0).value
    wait_time_after_tf_ready = node.declare_parameter('wait_time_after_tf_ready', 100.0).value

    framos_initialize = FramosInitialize(node, map_frame_name, camera_link_frame_name, rgb_camera_topic_name, depth_camera_topic_name, min_rgb_fps, min_depth_fps,
                                        max_buffer_size, wait_time_after_camera_ready, wait_time_after_tf_ready)

    while rclpy.ok():
        if framos_initialize.is_ready:
            sys.exit(0)
        rclpy.spin_once(node)


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)

if __name__ == "__main__":
    main()
