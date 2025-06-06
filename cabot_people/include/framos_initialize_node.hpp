// Copyright (c) 2025  IBM Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef CABOT_PEOPLE_FRAMOS_INITIALIZE_NODE_HPP_
#define CABOT_PEOPLE_FRAMOS_INITIALIZE_NODE_HPP_

#include <mutex>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace cabot_people
{

class FramosInitializeNode : public rclcpp::Node
{
public:
  explicit FramosInitializeNode(const rclcpp::NodeOptions & options);
  ~FramosInitializeNode();

  bool is_ready();

private:
  void call_enable_client(bool data);
  void rgb_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void depth_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void reset_framos();
  void check_transform();

  std::string map_frame_name_;
  std::string camera_link_frame_name_;
  std::string rgb_camera_topic_name_;
  std::string depth_camera_topic_name_;
  float min_rgb_fps_;
  float min_depth_fps_;
  int max_buffer_size_;
  float min_wait_after_camera_ready_;
  float max_wait_after_tf_ready_;

  bool is_ready_;
  bool is_reset_running_;
  double time_tf_ready_;
  double time_camera_ready_;
  std::queue<double> rgb_times_;
  std::queue<double> depth_times_;
  std::mutex rgb_times_mutex_;
  std::mutex depth_times_mutex_;

  tf2_ros::TransformListener * tf_listener_;
  tf2_ros::Buffer * tf_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_sub_;

  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_client_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cabot_people

#endif  // CABOT_PEOPLE_FRAMOS_INITIALIZE_NODE_HPP_
