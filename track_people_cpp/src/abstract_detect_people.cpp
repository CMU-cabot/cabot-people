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

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <tf2_eigen/tf2_eigen.h>

#include "abstract_detect_people.hpp"

namespace track_people_cpp
{
AbstractDetectPeople::AbstractDetectPeople(const std::string & node_name, rclcpp::NodeOptions options)
: Node(node_name, options),
  enable_detect_people_(true),
  queue_size_(2),
  estimate_ground_max_distance_(10.0),
  ransac_max_iteration_(10000),
  ransac_probability_(0.999),
  ransac_eps_angle_(5.0),
  ransac_input_min_height_(-0.20),
  ransac_input_max_height_(0.20),
  ransac_inlier_threshold_(0.01),
  ground_distance_threshold_(0.10),
  ignore_detect_ground_point_ratio_(0.90),
  debug_(false),
  parallel_(true),
  target_fps_(15.0),
  publish_detect_image_(false),
  fps_count_(0),
  detect_time_(0),
  detect_count_(0),
  depth_time_(0),
  depth_count_(0),
  is_model_ready_(false),
  is_camera_ready_(false),
  people_freq_(NULL),
  camera_freq_(NULL)
{
  cv::setNumThreads(0);

  // declare parameters
  remove_ground_ = this->declare_parameter("remove_ground", remove_ground_);
  detection_threshold_ = this->declare_parameter("detection_threshold", detection_threshold_);
  // minimum vertical size of box to consider a detection as a track
  minimum_detection_size_threshold_ = this->declare_parameter("minimum_detection_size_threshold", minimum_detection_size_threshold_);

  map_frame_name_ = this->declare_parameter("map_frame", map_frame_name_);
  robot_footprint_frame_name_ = this->declare_parameter("robot_footprint_frame", robot_footprint_frame_name_);
  camera_id_ = this->declare_parameter("camera_id", camera_id_);
  camera_link_frame_name_ = this->declare_parameter("camera_link_frame", camera_link_frame_name_);
  camera_info_topic_name_ = this->declare_parameter("camera_info_topic", camera_info_topic_name_);
  image_rect_topic_name_ = this->declare_parameter("image_rect_topic", image_rect_topic_name_);
  depth_registered_topic_name_ = this->declare_parameter("depth_registered_topic", depth_registered_topic_name_);
  depth_unit_meter_ = this->declare_parameter("depth_unit_meter", depth_unit_meter_);
  target_fps_ = this->declare_parameter("target_fps", target_fps_);
  publish_detect_image_ = this->declare_parameter("publish_detect_image", publish_detect_image_);

  // enable/disable service
  toggle_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "enable_detect_people",
    std::bind(&AbstractDetectPeople::enable_detect_people_cb, this, std::placeholders::_1, std::placeholders::_2));

  // TF
  tfBuffer = new tf2_ros::Buffer(this->get_clock());
  tfListener = new tf2_ros::TransformListener(*tfBuffer);

  // Subscriptions
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_name_, 10, std::bind(&AbstractDetectPeople::camera_info_cb, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "subscribe to %s and %s", image_rect_topic_name_.c_str(), depth_registered_topic_name_.c_str());
  rgb_image_sub_ = new message_filters::Subscriber<sensor_msgs::msg::Image>(this, image_rect_topic_name_);
  depth_image_sub_ = new message_filters::Subscriber<sensor_msgs::msg::Image>(this, depth_registered_topic_name_);
  rgb_depth_img_synch_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *rgb_image_sub_, *depth_image_sub_);
  rgb_depth_img_synch_->registerCallback(&AbstractDetectPeople::rgb_depth_img_cb, this);
  detected_boxes_pub_ = this->create_publisher<track_people_msgs::msg::TrackedBoxes>("/people/detected_boxes", 1);
  if (debug_) {
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("detect_ground", 1);
  }
  if (publish_detect_image_) {
    detect_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("detect_image", 1);
  }

  // process loop
  fps_loop_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  detect_loop_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  depth_loop_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  fps_loop_ = this->create_wall_timer(std::chrono::duration<float>(1.0 / target_fps_), std::bind(&AbstractDetectPeople::fps_loop_cb, this), fps_loop_cb_group_);
  detect_loop_ = this->create_wall_timer(std::chrono::duration<float>(0.01), std::bind(&AbstractDetectPeople::detect_loop_cb, this), detect_loop_cb_group_);
  depth_loop_ = this->create_wall_timer(std::chrono::duration<float>(0.01), std::bind(&AbstractDetectPeople::depth_loop_cb, this), depth_loop_cb_group_);

  // diagnostic updater
  updater_ = new diagnostic_updater::Updater(this);
  updater_->setHardwareID(this->get_namespace());
  diagnostic_updater::FrequencyStatusParam param1(&target_fps_, &target_fps_, 1.0, 2);
  camera_freq_ = new diagnostic_updater::HeaderlessTopicDiagnostic(std::string("CameraInput")+this->get_namespace(), *updater_, param1);
  diagnostic_updater::FrequencyStatusParam param2(&target_fps_, &target_fps_, 0.5, 2);
  people_freq_ = new diagnostic_updater::HeaderlessTopicDiagnostic(std::string("PeopleDetect")+this->get_namespace(), *updater_, param2);

  RCLCPP_INFO(this->get_logger(), "constructor completed");
}

void AbstractDetectPeople::enable_detect_people_cb(
  const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
{
  // TODO(daisukes): implementation
}

void AbstractDetectPeople::camera_info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  RCLCPP_INFO(this->get_logger(), "Got camera_info topic.");
  image_width_ = info->width;
  image_height_ = info->height;
  focal_length_ = info->k[0];
  center_x_ = info->k[2];
  center_y_ = info->k[5];
  camera_info_sub_.reset();
  RCLCPP_INFO(this->get_logger(), "Found camera_info topic.");

  // ROS_INFO("%d %d %.2f %.2f %.2f %.2f", image_width_, image_height_, focal_length_, focal_length_, center_x_, center_y_);
  pinhole_camera_intrinsic_ = open3d::camera::PinholeCameraIntrinsic(image_width_, image_height_, focal_length_, focal_length_, center_x_, center_y_);

  is_camera_ready_ = true;
}

/*
  this callback enqueue image data for process
 */
void AbstractDetectPeople::rgb_depth_img_cb(
  const sensor_msgs::msg::Image::SharedPtr & rgb_msg_ptr,
  const sensor_msgs::msg::Image::SharedPtr & depth_msg_ptr)
{
  if (!is_model_ready_ || !is_camera_ready_) {return;}

  try {
    DetectData dd;
    dd.header = rgb_msg_ptr->header;
    dd.transformStamped = tfBuffer->lookupTransform(
      map_frame_name_, camera_link_frame_name_, rgb_msg_ptr->header.stamp,
      std::chrono::duration<float>(1.0));

    // deal with rotation
    tf2::Transform pose_tf;
    tf2::fromMsg(dd.transformStamped.transform, pose_tf);
    tf2::Matrix3x3 mat(pose_tf.getRotation());
    double yaw, pitch, roll;
    mat.getRPY(roll, pitch, yaw);
    if (-M_PI / 4 < roll && roll < M_PI / 4) {
      dd.rotate = 0;
    } else if (M_PI / 4 <= roll && roll < M_PI / 4 * 3) {  // rotate 90 degree
      dd.rotate = 1;
    } else if (M_PI / 4 * 3 <= roll || roll <= -M_PI / 4 * 3) {  // up side down
      dd.rotate = 2;
    } else if (-M_PI / 4 * 3 < roll && roll <= -M_PI / 4) {  // rotate -90 degree
      dd.rotate = 3;
    }

    dd.rgb_msg_ptr = rgb_msg_ptr;
    dd.depth_msg_ptr = depth_msg_ptr;

    camera_freq_->tick();
    fps_count_++;
    temp_dd_ = std::make_shared<DetectData>(dd);

    {
      std::lock_guard<std::mutex> lock(queue_camera_mutex_);
      if (queue_camera_.size() < queue_size_) {
        queue_camera_.push(*temp_dd_);
      } else {
        queue_camera_.pop();
        queue_camera_.push(*temp_dd_);
      }
    }

    if (debug_) {
      int MAX = 100;

      if (fps_count_ == 0) {
        fps_time_ = std::chrono::high_resolution_clock::now();
      }
      if (fps_count_ == MAX) {
        auto now = std::chrono::high_resolution_clock::now();
        auto diff = static_cast<double>((now - fps_time_).count()) / 1000000000;
        RCLCPP_INFO(this->get_logger(), "fps %.2f (%.2f, %d)", fps_count_ / diff, diff, fps_count_);
        fps_count_ = 0;
        fps_time_ = std::chrono::high_resolution_clock::now();
      }

      if (detect_count_ == MAX) {
        RCLCPP_INFO(this->get_logger(), "detect %.2f fps %d", detect_count_ / detect_time_, detect_count_);
        detect_time_ = 0;
        detect_count_ = 0;

        RCLCPP_INFO(this->get_logger(), "depth %.2f fps %d", depth_count_ / depth_time_, depth_count_);
        depth_time_ = 0;
        depth_count_ = 0;
      }
    }
  } catch (std::exception & e) {
    RCLCPP_INFO(this->get_logger(), "tf2 error: %s", e.what());
  }
}

/*
  this callback try to keep the queue up to the target FPS
 */
void AbstractDetectPeople::fps_loop_cb()
{
  if (!is_model_ready_ || !is_camera_ready_) {return;}

  if (parallel_) {
    DetectData dd;
    {
      std::lock_guard<std::mutex> lock(queue_camera_mutex_);
      if (queue_camera_.size() == 0) {
        return;
      }
      dd = queue_camera_.front();
      queue_camera_.pop();
    }
    {
      std::lock_guard<std::mutex> lock(queue_ready_mutex_);
      if (queue_ready_.size() < queue_size_) {
        queue_ready_.push(dd);
      } else {
        queue_ready_.pop();
        queue_ready_.push(dd);
      }
    }
  } else {
    if (temp_dd_ == nullptr) {
      return;
    }

    if (people_freq_ != NULL) {
      people_freq_->tick();
    }
    DetectData dd = *temp_dd_;
    process_detect(dd);
    process_depth(dd);
    detected_boxes_pub_->publish(dd.result);
    if (publish_detect_image_) {
      publish_detect_image(dd);
    }
    temp_dd_ = nullptr;
  }
}

/*
  this callback process darknet detection as fast as possible if queue is not empty
 */
void AbstractDetectPeople::detect_loop_cb()
{
  if (!is_model_ready_ || !is_camera_ready_) {return;}

  DetectData dd;
  {
    std::lock_guard<std::mutex> lock(queue_ready_mutex_);
    if (queue_ready_.size() == 0) {
      return;
    }
    dd = queue_ready_.front();
    queue_ready_.pop();
  }

  auto start = std::chrono::high_resolution_clock::now();
  process_detect(dd);
  auto end = std::chrono::high_resolution_clock::now();
  detect_time_ += static_cast<double>((end - start).count()) / 1000000000;
  detect_count_++;

  std::lock_guard<std::mutex> lock(queue_detect_mutex_);
  if (queue_detect_.size() < queue_size_) {
    queue_detect_.push(dd);
  } else {
    queue_detect_.pop();
    queue_detect_.push(dd);
  }
}

/*
  this callback process depth estimation as fast as possible if queue is not empty
 */
void AbstractDetectPeople::depth_loop_cb()
{
  if (!is_model_ready_ || !is_camera_ready_) {return;}

  DetectData dd;
  {
    std::lock_guard<std::mutex> lock(queue_detect_mutex_);
    if (queue_detect_.size() == 0) {
      return;
    }
    dd = queue_detect_.front();
    queue_detect_.pop();
  }
  if (people_freq_ != NULL) {
    people_freq_->tick();
  }
  auto start = std::chrono::high_resolution_clock::now();
  process_depth(dd);
  detected_boxes_pub_->publish(dd.result);
  if (publish_detect_image_) {
    publish_detect_image(dd);
  }
  auto end = std::chrono::high_resolution_clock::now();
  depth_time_ += static_cast<double>((end - start).count()) / 1000000000;
  depth_count_++;
}

void AbstractDetectPeople::publish_detect_image(DetectData & dd)
{
  cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(dd.rgb_msg_ptr, sensor_msgs::image_encodings::BGR8);
  for (int i = 0; i < dd.result.tracked_boxes.size(); i++) {
    track_people_msgs::msg::TrackedBox tb = dd.result.tracked_boxes[i];
    rectangle(cv_rgb_ptr->image, cv::Point(tb.box.xmin, tb.box.ymin), cv::Point(tb.box.xmax, tb.box.ymax), cv::Scalar(0, 0, 255), 2);
    if (dd.masks.size()>0) {
      std::vector<cv::Mat> rgb_channels;
      cv::split(cv_rgb_ptr->image, rgb_channels);
      cv::Mat mask_img;
      if (dd.masks[i].cols==cv_rgb_ptr->image.cols && dd.masks[i].rows==cv_rgb_ptr->image.rows) {
        // rtmdet-inst
        mask_img = rgb_channels[0];
      } else {
        // maskrcnn
        int x0 = std::max(int(std::floor(tb.box.xmin)) - 1, 0);
        int y0 = std::max(int(std::floor(tb.box.ymin)) - 1, 0);
        mask_img = cv::Mat(rgb_channels[0], cv::Rect(x0, y0, dd.masks[i].cols, dd.masks[i].rows));
      }
      cv::bitwise_or(dd.masks[i], mask_img, mask_img);
      cv::merge(rgb_channels, cv_rgb_ptr->image);
    }
  }
  sensor_msgs::msg::Image::SharedPtr detect_image_msg = cv_rgb_ptr->toImageMsg();
  detect_image_pub_->publish(*detect_image_msg.get());
}

/*
  actual process of depth estimation
*/
void AbstractDetectPeople::process_depth(DetectData & dd)
{
  std::vector<track_people_msgs::msg::TrackedBox> & tracked_boxes = dd.result.tracked_boxes;
  tf2::Transform pose_tf;
  tf2::fromMsg(dd.transformStamped.transform, pose_tf);

  if (tracked_boxes.size() == 0) {
    return;
  }

  // ROS_INFO("depth_msg_ptr.use_count() %d", dd.depth_msg_ptr.use_count());
  auto cv_depth_ptr = cv_bridge::toCvShare(dd.depth_msg_ptr, sensor_msgs::image_encodings::TYPE_16UC1);
  // ROS_INFO("cv_depth_ptr.use_count() %d", cv_depth_ptr.use_count());
  auto depth_img = cv_depth_ptr->image;
  // ROS_INFO("depth_img %d %d", depth_img.cols, depth_img.rows);

  // calculate ground plane to ignore invalid detection
  pcl::ModelCoefficients::Ptr ground_coefficients(new pcl::ModelCoefficients);
  if (remove_ground_) {
    if (!camera_to_robot_footprint_matrix_) {
      // create transformation matrix from camera frame to robot footprint frame
      try {
        geometry_msgs::msg::TransformStamped camera_to_robot_footprint_msg = tfBuffer->lookupTransform(
          robot_footprint_frame_name_, camera_link_frame_name_,
          dd.depth_msg_ptr->header.stamp, std::chrono::duration<float>(1.0));

        Eigen::Isometry3d camera_to_robot_footprint_isometry3d = tf2::transformToEigen(camera_to_robot_footprint_msg);
        camera_to_robot_footprint_matrix_ = std::make_shared<Eigen::Matrix4d>(camera_to_robot_footprint_isometry3d.matrix());
      } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "TF is not ready");
        return;
      }
    }

    // ignore far depth points
    cv::threshold(depth_img, depth_img, estimate_ground_max_distance_ * 1000.0, 0, cv::THRESH_TOZERO_INV);

    // create depth point cloud in robot footprint frame
    open3d::geometry::Image o3d_depth_img;
    o3d_depth_img.Prepare(depth_img.cols, depth_img.rows, 1, 4);
    for (int row = 0; row < depth_img.rows; row++) {
      for (int col = 0; col < depth_img.cols; col++) {
        auto p = o3d_depth_img.PointerAt<float>(col, row);
        *p = static_cast<float>(depth_img.at<uint16_t>(row, col)) / 1000;
      }
    }
    auto o3d_pc = open3d::geometry::PointCloud::CreateFromDepthImage(o3d_depth_img, pinhole_camera_intrinsic_);

    // convert coordinate from x-right,y-down,z-forward coordinate to x-forward,y-left,z-up coordinate
    for (auto& pt : o3d_pc->points_) {
      pt = Eigen::Vector3d(pt.z(), -pt.x(), -pt.y());
    }

    // convert coordinate from camera to robot footprint
    auto o3d_pc_transform = o3d_pc->Transform(*camera_to_robot_footprint_matrix_);

    // select point cloud by height
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_transform(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : o3d_pc_transform.points_) {
      if (pt.z() >= ransac_input_min_height_ && pt.z() <= ransac_input_max_height_) {
        pcl_pc_transform->points.emplace_back(pt.x(), pt.y(), pt.z());
      }
    }

    // estimate ground plane
    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
    if (pcl_pc_transform->points.size() > 0) {
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setMaxIterations(ransac_max_iteration_);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(ransac_inlier_threshold_);
      seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
      seg.setEpsAngle(ransac_eps_angle_ * (M_PI / 180.0));
      seg.setProbability(ransac_probability_);
      seg.setInputCloud(pcl_pc_transform);
      seg.segment(*ground_inliers, *ground_coefficients);
    }

    // if ground plane is not found, set z=0 as ground plane
    if (ground_inliers->indices.size() == 0) {
      ground_coefficients->values.resize(4);
      ground_coefficients->values[0] = 0;
      ground_coefficients->values[1] = 0;
      ground_coefficients->values[2] = 1;
      ground_coefficients->values[3] = 0;
    }

    if (debug_) {
      pcl::PointIndices ground_indices;
      for (unsigned int i = 0; i < pcl_pc_transform->points.size(); i++) {
        const auto & p = pcl_pc_transform->points[i];
        double signed_distance = ground_coefficients->values[0] * p.x + ground_coefficients->values[1] * p.y + ground_coefficients->values[2] * p.z + ground_coefficients->values[3];
        if (abs(signed_distance) <= ground_distance_threshold_) {
          ground_indices.indices.push_back(i);
        }
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ground(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ExtractIndices<pcl::PointXYZ> ground_extract_indices;
      ground_extract_indices.setIndices(pcl::make_shared<const pcl::PointIndices>(ground_indices));
      ground_extract_indices.setInputCloud(pcl_pc_transform);
      ground_extract_indices.filter(*pcl_ground);

      sensor_msgs::msg::PointCloud2 ros_ground;
      pcl::toROSMsg(*pcl_ground, ros_ground);
      ros_ground.header.stamp = dd.depth_msg_ptr->header.stamp;
      ros_ground.header.frame_id = robot_footprint_frame_name_;
      ground_pub_->publish(ros_ground);
    }
  }

  for (auto it = tracked_boxes.begin(); it != tracked_boxes.end(); ) {
    std::shared_ptr<open3d::geometry::PointCloud> pc;
    if (dd.masks.size()>0) {
      size_t idx = std::distance(tracked_boxes.begin(), it);
      pc = generatePointCloudFromDepthAndMask(depth_img, it->box, dd.masks[idx]);
    } else {
      pc = generatePointCloudFromDepthAndBox(depth_img, it->box);
    }
    if (!pc->HasPoints()) {
      RCLCPP_INFO(this->get_logger(), "no points found");
      tracked_boxes.erase(it);
      continue;
    }
    if (remove_ground_) {
      // convert coordinate from x-right,y-down,z-forward coordinate to x-forward,y-left,z-up coordinate
      open3d::geometry::PointCloud pc_x_forward;
      for (auto& pt : pc->points_) {
        pc_x_forward.points_.emplace_back(pt.z(), -pt.x(), -pt.y());
      }

      // convert coordinate from camera to robot footprint
      auto pc_x_forward_transform = pc_x_forward.Transform(*camera_to_robot_footprint_matrix_);

      // ignore the detection result if the ratio of points that are close to ground is large
      int ground_point_num = 0;
      for (auto p : pc_x_forward_transform.points_) {
        double signed_distance = ground_coefficients->values[0] * p[0] + ground_coefficients->values[1] * p[1] + ground_coefficients->values[2] * p[2] + ground_coefficients->values[3];
        if (abs(signed_distance) <= ground_distance_threshold_) {
          ground_point_num += 1;
        }
      }
      double ground_point_ratio = ground_point_num / pc_x_forward_transform.points_.size();
      if (ground_point_ratio > ignore_detect_ground_point_ratio_) {
        RCLCPP_INFO(this->get_logger(), "ignore detection on ground, ground point ratio %f", ground_point_ratio);
        tracked_boxes.erase(it);
        continue;
      }
    }

    auto median = getMedianOfPoints(*pc);

    if (median.hasNaN()) {
      RCLCPP_INFO(this->get_logger(), "median has NAN");
      tracked_boxes.erase(it);
      continue;
    }

    // convert realsense coordinate (x:left-right, y:top-down,   z:back-front) to
    //               ROS coordinate (x:back-front, y:right-left, z:down-top)
    double x = median(2);
    double y = -median(0);
    double z = 0;  // do not use height

    // if camera is sideway: roll = 90 or 270 degree
    if (dd.rotate % 2 == 1) {
      y = 0;
      z = -median(1);
    }

    tf2::Transform median_tf(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(x, y, z));
    auto map_center = (pose_tf * median_tf).getOrigin();

    it->center3d.x = map_center.x();
    it->center3d.y = map_center.y();
    it->center3d.z = map_center.z();
    it++;
  }
}

std::shared_ptr<open3d::geometry::PointCloud> AbstractDetectPeople::generatePointCloudFromDepthAndBox(
  cv::Mat & depth_img, track_people_msgs::msg::BoundingBox & box)
{
  auto o3d_depth_ptr = std::make_shared<open3d::geometry::Image>();
  o3d_depth_ptr->Prepare(depth_img.cols, depth_img.rows, 1 /* channel */, 4 /* bytes per channel */);  // convert to float image
  fill(o3d_depth_ptr->data_.begin(), o3d_depth_ptr->data_.end(), 0);

  for (int row = box.ymin; row < box.ymax; row++) {
    for (int col = box.xmin; col < box.xmax; col++) {
      auto p = o3d_depth_ptr->PointerAt<float>(col, row);
      *p = static_cast<float>(depth_img.at<uint16_t>(row, col)) / 1000;
    }
  }

  return open3d::geometry::PointCloud::CreateFromDepthImage(*o3d_depth_ptr, pinhole_camera_intrinsic_);
}

std::shared_ptr<open3d::geometry::PointCloud> AbstractDetectPeople::generatePointCloudFromDepthAndMask(
  cv::Mat & depth_img, track_people_msgs::msg::BoundingBox & box, cv::Mat & mask)
{
  auto o3d_depth_ptr = std::make_shared<open3d::geometry::Image>();
  o3d_depth_ptr->Prepare(depth_img.cols, depth_img.rows, 1 /* channel */, 4 /* bytes per channel */);  // convert to float image
  fill(o3d_depth_ptr->data_.begin(), o3d_depth_ptr->data_.end(), 0);

  std::vector<cv::Point> mask_points;
  cv::Mat mask_img;
  if (mask.cols==depth_img.cols && mask.rows==depth_img.rows) {
    // rtmdet-inst
    mask_img = mask;
  } else {
    // maskrcnn
    int x0 = std::max(int(std::floor(box.xmin)) - 1, 0);
    int y0 = std::max(int(std::floor(box.ymin)) - 1, 0);
    mask_img = cv::Mat::zeros(depth_img.size(), CV_8UC1);
    mask.copyTo(mask_img(cv::Rect(x0, y0, mask.cols, mask.rows)));
  }
  cv::findNonZero(mask_img, mask_points);
  for (auto mask_point : mask_points) {
    auto p = o3d_depth_ptr->PointerAt<float>(mask_point.x, mask_point.y);
    *p = static_cast<float>(depth_img.at<uint16_t>(mask_point.y, mask_point.x)) / 1000;
  }

  return open3d::geometry::PointCloud::CreateFromDepthImage(*o3d_depth_ptr, pinhole_camera_intrinsic_);
}

Eigen::Vector3d AbstractDetectPeople::getMedianOfPoints(open3d::geometry::PointCloud & pc)
{
  Eigen::Vector3d ret;

  auto & ps = pc.points_;
  for (int i = 0; i < 3; i++) {
    // partially sort until median, 50 percentile
    std::nth_element(
      ps.begin(), ps.begin() + ps.size() / 2, ps.end(),
      [i](Eigen::Vector3d & a, Eigen::Vector3d & b) {return a(i) < b(i);});
    ret[i] = ps[ps.size() / 2][i];
  }

  return ret;
}

}  // namespace track_people_cpp
