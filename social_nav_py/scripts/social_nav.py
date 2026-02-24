#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import base64
import os
import sys
import time
import re
import numpy as np

from openai import OpenAI
# import requests # Not used in code

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from track_people_msgs.msg import TrackedBoxes
from social_nav_msgs.msg import SocialNavMsg


# OpenAI API Key
AZURE_OPENAI_API_KEY = os.getenv('AZURE_OPENAI_API_KEY')
AZURE_OPENAI_BASEURL = os.getenv('AZURE_OPENAI_BASEURL')
VLM_SOCIAL_NAV_MODEL = os.getenv('VLM_SOCIAL_NAV_MODEL', 'gpt-4o')

HEAD_DIR = ['left', 'straight', 'right']
SPEED = ['slow down', 'speed up', 'maintain', 'stop'] 

class SocialNavGPT(Node):
    def __init__(self):
        super().__init__('social_nav')
        
        # RGB img
        self.scene_data = None
        # ROS msg
        self.scene_msg = None
        # CV img
        self.cv_image = None

        self.done = False
        self.answer = None
        self.verbose = True
        self.openai_model = VLM_SOCIAL_NAV_MODEL
        # robot state
        self.ego_state = None
        self.ego_state_string = (1.0, 'straight')

        self.br = CvBridge()

        self.declare_parameter('detect_size', 150) # Increased threshold
        self.detect_size = self.get_parameter('detect_size').value
        
        self.declare_parameter('input_image_topic', '/camera/color/image_raw')
        input_image_topic = self.get_parameter('input_image_topic').value
        
        self.declare_parameter('people_topic', '/people')
        people_topic = self.get_parameter('people_topic').value

        self.declare_parameter('view_image', True)
        self.view_image = self.get_parameter('view_image').value

        # Heuristic for compressed topic
        self.compressed_input = "compressed" in input_image_topic

        # Subscribers
        if self.compressed_input:
            self.create_subscription(
                CompressedImage, input_image_topic, self.image_callback, 1
            )
        else:
            self.create_subscription(
                Image, input_image_topic, self.image_callback, 1
            )
        
        # Subscribe to people result (assuming people is running somewhere else)
        self.create_subscription(TrackedBoxes, people_topic, self.people_callback, 1)
        
        self.v2s_pub = self.create_publisher(SocialNavMsg, '/openai_result', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 1)

        # OpenAI
        if AZURE_OPENAI_API_KEY and AZURE_OPENAI_BASEURL:
             self.client = OpenAI(
                api_key=AZURE_OPENAI_API_KEY,
                base_url=AZURE_OPENAI_BASEURL
            )
        else:
             self.get_logger().warn("AZURE_OPENAI environment variables not set. OpenAI features will fail.")
             self.client = OpenAI(api_key="dummy") # Prevent crash on init if missing

        self.ans = None

        # PROMPT
        self.prompt = f"""
        Task:
        The image depicts your current view while you navigating towards the goal. How will you navigate concerning the person in your view? 
        You will need to follow general walking etiquette.

        Ego state:
        - heading direction: {self.ego_state_string[1]}
        - linear velocity: {self.ego_state_string[0]}

        Remember:
        - Move to the right when passing by a person.
        - Pass on the left when overtaking another person.
        - Do not obstruct others' paths.
        - Stop and wait when a person is crossing perpendicularly.
        - Stop and wait on doorways.

        Answer Format (STRICT):
        Return exactly one line in this format with no extra words or punctuation:
        Move <heading> with <speed>
        - <heading> must be one of: left, straight, right
        - <speed> must be one of: slow down, speed up, maintain, stop
        """

    # Function to encode tvisihe image
    def encode_image(self, img):
        if self.compressed_input:
            # For compressed image, the data is already bytes of jpeg/png usually
            return base64.b64encode(img).decode('utf-8')
        else:
            if self.cv_image is None:
                return ""
            _, img_encoded = cv2.imencode('.jpeg', self.cv_image)
            return base64.b64encode(img_encoded).decode('utf-8')


    def openai(self, img):
        if not img:
            return False
            
        try:
            if self.verbose:
                self.get_logger().info(f"OpenAI model: {self.openai_model}")
                self.get_logger().info(f"OpenAI prompt: {self.prompt}")
            response = self.client.chat.completions.create(
                model=self.openai_model,
                messages=[
                    {
                        "role": "user",
                        "content": [{"type": "text", "text": f"{self.prompt}"},
                                    {"type": "image_url",
                                     "image_url": {
                                        "url": f"data:image/jpeg;base64,{img}",
                                        "detail": "low"},
                                    }
                                   ],
                    }
                ],
                temperature = 0.0,
                max_tokens = 300,
            )
            self.answer = response.choices[0].message.content
            if self.verbose:
                self.get_logger().info(f"OpenAI response: {self.answer}")

            return self.answer_to_message(self.answer)
        except Exception as e:
            self.get_logger().error(f"OpenAI call failed: {e}")
            return False

    def parse_values(self, string):
        cleaned = self.normalize_answer(string)
        heading_pattern = r"\bmove\s+(\w+)"
        speed_pattern = r"\bwith\s+(\w+(?:\s+\w+)*)"

        heading_match = re.search(heading_pattern, cleaned)
        speed_match = re.search(speed_pattern, cleaned)

        if heading_match and speed_match:
            heading_direction = heading_match.group(1)
            speed = speed_match.group(1).strip()
            if speed.endswith(" speed"):
                speed = speed[:-6].strip()
            return heading_direction, speed
        else:
            return None, None

    def normalize_answer(self, text):
        if not text:
            return ""
        cleaned = text.lower()
        # strip markdown emphasis and punctuation noise
        cleaned = cleaned.replace("*", "")
        cleaned = re.sub(r"[^\w\s]", " ", cleaned)
        cleaned = re.sub(r"\s+", " ", cleaned).strip()
        return cleaned

    def answer_to_message(self, ans):
        if not ans:
            return False
        a1, a2 = self.parse_values(ans)
        
        try:
             idx1 = HEAD_DIR.index(a1)
             idx2 = SPEED.index(a2)
             self.get_logger().info(f"Parsed: {idx1}, {idx2}")
        except ValueError:
             self.get_logger().warn(f"Failed to parse answer values: {a1}, {a2}")
             return False

        if a1 != None and a2 != None:
            msg = SocialNavMsg()
            msg.head_dir = str(idx1)
            msg.speed = str(idx2)
            self.v2s_pub.publish(msg)

            return True
        return False

    def people_callback(self, msg):
        for tracked_box in msg.tracked_boxes:
            box = tracked_box.box
            bbox_height = box.ymax - box.ymin
            
            if box.class_name == "person" and bbox_height > self.detect_size:
                if self.scene_data is not None:
                    scene_base64 = self.encode_image(self.scene_data)
                    
                    start = time.time()
                    success = self.openai(scene_base64)
                    end = time.time()

                    if success:
                        # Wait a bit after successful command to avoid spamming
                        time.sleep(2)
                        msg = SocialNavMsg()
                        msg.head_dir = str(-1)
                        msg.speed = str(-1)
                        self.v2s_pub.publish(msg)
                        self.get_logger().info("[-1, -1] published")
                        return # Process only one person at a time (closest/largest)

    def image_callback(self, msg):
        self.scene_msg = msg
        self.scene_data = msg.data # For compressed, this is data. For Image, this is data.

        try:
            if self.compressed_input:
                self.cv_image = self.br.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                self.cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        if not self.done:
            if self.view_image and self.cv_image is not None:
                cv2.imshow("GPT4V", self.cv_image)
                cv2.waitKey(1)

    def odom_callback(self, msg):
        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z
        self.ego_state = (linear, angular)

        if abs(linear) < 0.05:
            self.ego_state_string = (linear, 'straight')
        elif linear < 0.05:
            self.ego_state_string = (linear, 'right')
        else:
            self.ego_state_string = (linear, 'left')


def main(args=None):
    rclpy.init(args=args)
    sng = SocialNavGPT()
    rclpy.spin(sng)
    sng.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
