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

#include "framos_initialize_node.hpp"


namespace cabot_people
{

FramosInitializeNode::FramosInitializeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("framos_initialize_node", options),
  map_frame_name_("map"),
  camera_link_frame_name_("camera_link"),
  rgb_camera_topic_name_("color/camera_info"),
  depth_camera_topic_name_("aligned_depth_to_color/camera_info"),
  min_rgb_fps_(10.0),
  min_depth_fps_(10.0),
  max_buffer_size_(100),
  min_wait_after_camera_ready_(5.0),
  max_wait_after_tf_ready_(100.0),
  is_ready_(false),
  time_tf_ready_(0.0),
  time_camera_ready_(0.0)
{
  map_frame_name_ = this->declare_parameter("map_frame", map_frame_name_);
  camera_link_frame_name_ = this->declare_parameter("camera_link_frame", camera_link_frame_name_);
  rgb_camera_topic_name_ = this->declare_parameter("rgb_camera_topic_name", rgb_camera_topic_name_);
  depth_camera_topic_name_ = this->declare_parameter("depth_camera_topic_name", depth_camera_topic_name_);
  min_rgb_fps_ = this->declare_parameter("min_rgb_fps", min_rgb_fps_);
  min_depth_fps_ = this->declare_parameter("min_depth_fps", min_depth_fps_);
  max_buffer_size_ = this->declare_parameter("max_buffer_size", max_buffer_size_);
  min_wait_after_camera_ready_ = this->declare_parameter("min_wait_after_camera_ready", min_wait_after_camera_ready_);
  max_wait_after_tf_ready_ = this->declare_parameter("max_wait_after_tf_ready", max_wait_after_tf_ready_);

  tf_buffer_ = new tf2_ros::Buffer(get_clock());
  tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_, this);

  rgb_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(rgb_camera_topic_name_, 10, std::bind(&FramosInitializeNode::rgb_cb, this, std::placeholders::_1));
  depth_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(depth_camera_topic_name_, 10, std::bind(&FramosInitializeNode::depth_cb, this, std::placeholders::_1));

  set_parameters_client_ = this->create_client<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically");
  set_parameters_client_->wait_for_service();

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0),
    std::bind(&FramosInitializeNode::check_transform, this));
}

FramosInitializeNode::~FramosInitializeNode()
{
  delete tf_listener_;
  delete tf_buffer_;
}

bool FramosInitializeNode::is_ready()
{
  return is_ready_;
}

void FramosInitializeNode::rgb_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  double now = rclcpp::Time(msg->header.stamp).seconds();
  if (time_camera_ready_ <= 0) {
    time_camera_ready_ = now;
  } else if ((now - time_camera_ready_) > min_wait_after_camera_ready_) {
    std::lock_guard<std::mutex> rgb_times_lock(rgb_times_mutex_);
    if (rgb_times_.size() >= max_buffer_size_) {
      rgb_times_.pop();
    }
    rgb_times_.push(now);
  }
}

void FramosInitializeNode::depth_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  double now = rclcpp::Time(msg->header.stamp).seconds();
  if (time_camera_ready_ <= 0) {
    time_camera_ready_ = now;
  } else if ((now - time_camera_ready_) > min_wait_after_camera_ready_) {
    std::lock_guard<std::mutex> depth_times_lock(depth_times_mutex_);
    if (depth_times_.size() >= max_buffer_size_) {
      depth_times_.pop();
    }
    depth_times_.push(now);
  }
}

void FramosInitializeNode::reset_framos()
{
  auto enable_done_callback = [this](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture response) {
    if (response.valid() && response.get()->result.successful) {
      RCLCPP_INFO(this->get_logger(), "Successed to enable FRAMOS");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to enable FRAMOS");
    }
    is_reset_running_ = false;
  };

  auto disable_done_callback = [this, enable_done_callback](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture response) {
    if (response.valid() && response.get()->result.successful) {
      RCLCPP_INFO(this->get_logger(), "Successed to disable FRAMOS");

      std::vector<rcl_interfaces::msg::Parameter> params;
      rcl_interfaces::msg::Parameter rgb_param;
      rgb_param.name = "enable_color";
      rgb_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      rgb_param.value.bool_value = true;
      params.push_back(rgb_param);
      rcl_interfaces::msg::Parameter depth_param;
      depth_param.name = "enable_depth";
      depth_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      depth_param.value.bool_value = true;
      params.push_back(depth_param);

      auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
      request->parameters = params;
      set_parameters_client_->async_send_request(request, enable_done_callback);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to disable FRAMOS");
      is_reset_running_ = false;
    }
  };

  is_reset_running_ = true;

  time_tf_ready_ = 0.0;
  time_camera_ready_ = 0.0;
  std::queue<double>().swap(rgb_times_);
  std::queue<double>().swap(depth_times_);

  std::vector<rcl_interfaces::msg::Parameter> params;
  rcl_interfaces::msg::Parameter rgb_param;
  rgb_param.name = "enable_color";
  rgb_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  rgb_param.value.bool_value = false;
  params.push_back(rgb_param);
  rcl_interfaces::msg::Parameter depth_param;
  depth_param.name = "enable_depth";
  depth_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  depth_param.value.bool_value = false;
  params.push_back(depth_param);

  auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
  request->parameters = params;
  set_parameters_client_->async_send_request(request, disable_done_callback);
}

void FramosInitializeNode::check_transform()
{
  if (is_ready_) {
    RCLCPP_INFO(this->get_logger(), "Framos is ready");
    rclcpp::shutdown();
    std::exit(EXIT_SUCCESS);
  }

  // check tf because launching FRAMOS before initial localization often fails on kx models
  double now = rclcpp::Time(get_clock()->now()).seconds();
  try {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      map_frame_name_, camera_link_frame_name_,
      rclcpp::Time(0), rclcpp::Duration(std::chrono::duration<double>(1.0)));
    if (time_tf_ready_ <= 0) {
      time_tf_ready_ = now;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "TF is not ready");
    time_tf_ready_ = 0.0;
    return;
  }

  // calculate FPS
  int rgb_buffer_size = 0;
  int depth_buffer_size = 0;
  double rgb_fps = 0.0;
  double depth_fps = 0.0;
  {
    std::lock_guard<std::mutex> rgb_times_lock(rgb_times_mutex_);
    std::lock_guard<std::mutex> depth_times_lock(depth_times_mutex_);
    rgb_buffer_size = rgb_times_.size();
    depth_buffer_size = depth_times_.size();
    if ((rgb_buffer_size == max_buffer_size_) && (depth_buffer_size == max_buffer_size_)) {
      double rgb_buffer_time = rgb_times_.back() - rgb_times_.front();
      if (rgb_buffer_time > 0) {
        rgb_fps = rgb_buffer_size / rgb_buffer_time;
      }
      double depth_buffer_time = depth_times_.back() - depth_times_.front();
      if (depth_buffer_time > 0) {
        depth_fps = depth_buffer_size / depth_buffer_time;
      }
    }
  }

  if ((rgb_buffer_size == max_buffer_size_) && (depth_buffer_size == max_buffer_size_)) {
    // reset FRAMOS if FPS is lower than threshold
    if ((rgb_fps >= min_rgb_fps_) && (depth_fps >= min_depth_fps_)) {
      RCLCPP_INFO(this->get_logger(), "FRAMOS is ready, RGB FPS: %.2f, depth FPS: %.2f", rgb_fps, depth_fps);
      is_ready_ = true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Reset FRAMOS because FPS is low, RGB FPS: %.2f, depth FPS: %.2f", rgb_fps, depth_fps);
      reset_framos();
    }
  } else if ((now - time_tf_ready_) > max_wait_after_tf_ready_) {
    // reset FRAMOS if buffers are not filled for a long time
    RCLCPP_WARN(this->get_logger(), "Reset FRAMOS because FRAMOS seems not working");
    reset_framos();
  }
}

}  // namespace cabot_people

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cabot_people::FramosInitializeNode>(rclcpp::NodeOptions());
  while (rclcpp::ok()) {
    if (node->is_ready()) {
      rclcpp::shutdown();
      break;
    }
    rclcpp::spin_some(node);
  }
  return 0;
}
