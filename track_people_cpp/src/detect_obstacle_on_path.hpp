// Copyright (c) 2022  Carnegie Mellon University
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
//
// Author: Daisuke Sato <daisukes@cmu.edu>

#ifndef DETECT_OBSTACLE_ON_PATH_HPP_
#define DETECT_OBSTACLE_ON_PATH_HPP_

#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/flann/flann.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <track_people_msgs/msg/tracked_boxes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace track_people_cpp
{
class DetectObstacleOnPath : public rclcpp::Node
{
public:
  explicit DetectObstacleOnPath(rclcpp::NodeOptions options);

private:
  void update();
  void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void planCallback(nav_msgs::msg::Path::SharedPtr msg);

  std::string sensor_id_;
  std::string scan_topic_;
  std::string map_frame_name_;
  std::string robot_frame_name_;
  tf2_ros::TransformListener * tfListener;
  tf2_ros::Buffer * tfBuffer;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Publisher<track_people_msgs::msg::TrackedBoxes>::SharedPtr obstacle_pub_;

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  nav_msgs::msg::Path::SharedPtr last_plan_;
  uint64_t last_pose_index_;

  cv::Mat * data_;
  cv::flann::Index * idx_;

  float footprint_size_;
  float safety_margin_;
  double target_fps_;

  diagnostic_updater::Updater * updater_;
  diagnostic_updater::HeaderlessTopicDiagnostic * obstacle_freq_;
};

}  // namespace track_people_cpp

#endif  // DETECT_OBSTACLE_ON_PATH_HPP_
