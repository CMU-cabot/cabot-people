#!/usr/bin/env python3

# Copyright (c) 2024  Carnegie Mellon University
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

import rclpy
from rclpy.node import Node
from track_people_msgs.msg import TrackedBoxes

class FilterTrackedBoxes(Node):
    def __init__(self):
        super().__init__('filter_tracked_boxes')
        
        self.target_camera_id = self.declare_parameter('target_camera_id', '').value
        self.input_topic = self.declare_parameter('input_topic', '/people/detected_boxes').value
        self.output_topic = self.declare_parameter('output_topic', '/people/detected_boxes_filtered').value
        
        if not self.target_camera_id:
            self.get_logger().warn("target_camera_id is not set. All messages will be filtered out.")
            
        self.sub = self.create_subscription(TrackedBoxes, self.input_topic, self.callback, 10)
        self.pub = self.create_publisher(TrackedBoxes, self.output_topic, 10)
        
        self.get_logger().info(f"Filtering {self.input_topic} for camera_id='{self.target_camera_id}' -> {self.output_topic}")
        
    def callback(self, msg):
        if msg.camera_id == self.target_camera_id:
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FilterTrackedBoxes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()