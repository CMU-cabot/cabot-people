// Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

#ifndef DETECT_ABSTRACT_PEOPLE_HPP_
#define DETECT_ABSTRACT_PEOPLE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/geometry/Image.h>
#include <open3d/geometry/PointCloud.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <track_people_msgs/msg/bounding_box.hpp>
#include <track_people_msgs/msg/tracked_box.hpp>
#include <track_people_msgs/msg/tracked_boxes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace track_people_cpp
{
struct DetectData
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::Image::ConstPtr rgb_msg_ptr;
  sensor_msgs::msg::Image::ConstPtr depth_msg_ptr;
  geometry_msgs::msg::TransformStamped transformStamped;
  int rotate;
  track_people_msgs::msg::TrackedBoxes result;
  std::vector<cv::Mat> masks;
};

class AbstractDetectPeople : public rclcpp::Node
{
public:
  explicit AbstractDetectPeople(const std::string & node_name, rclcpp::NodeOptions options);

protected:
  virtual void process_detect(DetectData & dd)=0;

  bool debug_;
  bool parallel_;
  bool is_model_ready_;
  bool is_camera_ready_;

  // config parameters
  bool remove_ground_;
  double detection_threshold_;
  double minimum_detection_size_threshold_;
  std::string map_frame_name_;
  std::string robot_footprint_frame_name_;
  std::string camera_id_;
  std::string camera_link_frame_name_;
  std::string camera_info_topic_name_;
  std::string image_rect_topic_name_;
  std::string depth_registered_topic_name_;
  bool depth_unit_meter_;
  double target_fps_;
  bool publish_detect_image_;

  // image config
  int image_width_;
  int image_height_;
  double focal_length_;
  double center_x_;
  double center_y_;
  open3d::camera::PinholeCameraIntrinsic pinhole_camera_intrinsic_;
  std::shared_ptr<Eigen::Matrix4d> camera_to_robot_footprint_matrix_;

private:
  void enable_detect_people_cb(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);
  void camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr info);
  void rgb_depth_img_cb(
    const sensor_msgs::msg::Image::SharedPtr & rgb_msg_ptr,
    const sensor_msgs::msg::Image::SharedPtr & depth_msg_ptr);
  void fps_loop_cb();
  void detect_loop_cb();
  void depth_loop_cb();
  void publish_detect_image(DetectData & dd);
  void process_depth(DetectData & dd);
  std::shared_ptr<open3d::geometry::PointCloud> generatePointCloudFromDepthAndBox(
    cv::Mat & depth_img, track_people_msgs::msg::BoundingBox & box);
  std::shared_ptr<open3d::geometry::PointCloud> generatePointCloudFromDepthAndMask(
    cv::Mat & depth_img, track_people_msgs::msg::BoundingBox & box, cv::Mat & mask);
  Eigen::Vector3d getMedianOfPoints(open3d::geometry::PointCloud & pc);

  std::shared_ptr<DetectData> temp_dd_;
  std::queue<DetectData> queue_camera_;
  std::queue<DetectData> queue_ready_;
  std::queue<DetectData> queue_detect_;
  int queue_size_;
  std::mutex queue_camera_mutex_;
  std::mutex queue_ready_mutex_;
  std::mutex queue_detect_mutex_;

  double estimate_ground_max_distance_;
  int ransac_max_iteration_;
  double ransac_probability_;
  double ransac_eps_angle_;
  double ransac_input_min_height_;
  double ransac_input_max_height_;
  double ransac_inlier_threshold_;
  double ground_distance_threshold_;
  double ignore_detect_ground_point_ratio_;

  bool enable_detect_people_;

  std::chrono::time_point<std::chrono::high_resolution_clock> fps_time_;
  int fps_count_;
  double detect_time_;
  int detect_count_;
  double depth_time_;
  int depth_count_;

  tf2_ros::TransformListener * tfListener;
  tf2_ros::Buffer * tfBuffer;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_srv_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<track_people_msgs::msg::TrackedBoxes>::SharedPtr detected_boxes_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detect_image_pub_;
  rclcpp::CallbackGroup::SharedPtr fps_loop_cb_group_;
  rclcpp::CallbackGroup::SharedPtr detect_loop_cb_group_;
  rclcpp::CallbackGroup::SharedPtr depth_loop_cb_group_;
  rclcpp::TimerBase::SharedPtr fps_loop_;
  rclcpp::TimerBase::SharedPtr detect_loop_;
  rclcpp::TimerBase::SharedPtr depth_loop_;

  message_filters::Subscriber<sensor_msgs::msg::Image> * rgb_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> * depth_image_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> * rgb_depth_img_synch_;

  diagnostic_updater::Updater * updater_;
  diagnostic_updater::HeaderlessTopicDiagnostic * people_freq_;
  diagnostic_updater::HeaderlessTopicDiagnostic * camera_freq_;
};

}  // namespace track_people_cpp

#endif  // DETECT_ABSTRACT_PEOPLE_HPP_
