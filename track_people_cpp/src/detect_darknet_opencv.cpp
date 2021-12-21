// Copyright (c) 2021  Carnegie Mellon University
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

#include "detect_darknet_opencv.hpp"

namespace TrackPeopleCPP
{
  DetectDarknetOpencv::DetectDarknetOpencv()
    : enable_detect_people_(true),
      queue_size_(3),
      debug_(false),
      parallel_(true),
      fps_count_(0),
      detect_time_(0),
      detect_count_(0),
      depth_time_(0),
      depth_count_(0),
      is_ready_(false)
  {

    if (debug_) {
      cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
    }
  }

  void DetectDarknetOpencv::onInit(ros::NodeHandle &nh)
  {
    nh.getParam("/track_people_py/detection_threshold", detection_threshold_);
    // minimum vertical size of box to consider a detection as a track
    nh.getParam("/track_people_py/minimum_detection_size_threshold", minimum_detection_size_threshold_);

    nh.getParam("/track_people_py/detect_config_file", detect_config_filename_);
    nh.getParam("/track_people_py/detect_weight_file", detect_weight_filename_);
    //nh.getParam("/track_people_py/detect_label_file", detect_label_filename_);

    ROS_INFO("weights: %s", detect_weight_filename_.c_str());
    ROS_INFO("config : %s", detect_config_filename_.c_str());
    darknet_ = cv::dnn::readNet(detect_weight_filename_, detect_config_filename_, "Darknet");
    darknet_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    darknet_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);

    model_ = std::make_shared<cv::dnn::DetectionModel>(cv::dnn::DetectionModel(darknet_));
    model_->setInputParams(1/255.0, cv::Size(416, 416), cv::Scalar(), true);

    cv::Mat dummy = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
    std::vector<int> classIds;
    std::vector<float> scores;
    std::vector<cv::Rect> boxes;
    model_->detect(dummy, classIds, scores, boxes, 0.6, 0.4);
    ROS_INFO("Model Loaded");

    nh.getParam("/track_people_py/map_frame", map_frame_name_);
    nh.getParam("/track_people_py/camera_id", camera_id_);
    nh.getParam("/track_people_py/camera_link_frame", camera_link_frame_name_);
    nh.getParam("/track_people_py/camera_info_topic", camera_info_topic_name_);
    nh.getParam("/track_people_py/image_rect_topic", image_rect_topic_name_);
    nh.getParam("/track_people_py/depth_registered_topic", depth_registered_topic_name_);
    nh.getParam("/track_people_py/depth_unit_meter", depth_unit_meter_);
    nh.getParam("/track_people_py/target_fps", target_fps_);
      
    toggle_srv_ = nh.advertiseService("/enable_detect_people",
				      &DetectDarknetOpencv::enable_detect_people_cb, this);
      
    //ROS_INFO("Waiting for camera_info topic...");
    //ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_name_, nh);
    //ROS_INFO("Found camera_info topic.");

    camera_info_sub_ = nh.subscribe(camera_info_topic_name_, 10,
				    &DetectDarknetOpencv::camera_info_cb, this);

    tfListener = new tf2_ros::TransformListener(tfBuffer);
      
    ROS_INFO("subscribe to %s and %s", image_rect_topic_name_.c_str(), depth_registered_topic_name_.c_str());
    rgb_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, image_rect_topic_name_, 10);
    depth_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, depth_registered_topic_name_, 10);
    rgb_depth_img_synch_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *rgb_image_sub_, *depth_image_sub_);
    rgb_depth_img_synch_->registerCallback(&DetectDarknetOpencv::rgb_depth_img_cb, this);
      
    detected_boxes_pub_ = nh.advertise<track_people_py::TrackedBoxes>("/track_people_py/detected_boxes", 1);

    fps_loop_ = nh.createTimer(ros::Duration(1.0 / target_fps_), &DetectDarknetOpencv::fps_loop_cb, this);

    //detect_loop_ = nh.createTimer(ros::Duration(0.01), &DetectDarknetOpencv::detect_loop_cb, this);
    //depth_loop_ = nh.createTimer(ros::Duration(0.01), &DetectDarknetOpencv::depth_loop_cb, this);

    detect_thread_ = std::thread([](DetectDarknetOpencv *obj){
	ros::TimerEvent event;
	while(ros::ok()) {
	  if (obj->is_ready_) {
	    obj->detect_loop_cb(event);
	  }
	  std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
      }, this);
    
    depth_thread_ = std::thread([](DetectDarknetOpencv *obj){
	ros::TimerEvent event;
	while(ros::ok()) {
	  if (obj->is_ready_) {
	    obj->depth_loop_cb(event);
	  }
	  std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
      }, this);
  
  }

  bool DetectDarknetOpencv::enable_detect_people_cb(std_srvs::SetBool::Request &req,
						    std_srvs::SetBool::Response &res) {
  }

  void DetectDarknetOpencv::camera_info_cb(const sensor_msgs::CameraInfoConstPtr& info) {
    ROS_INFO("Got camera_info topic.");
    image_width_ = info->width;
    image_height_ = info->height;
    focal_length_ = info->K[0];
    center_x_ = info->K[2];
    center_y_ = info->K[5];
    camera_info_sub_.shutdown();
    ROS_INFO("Found camera_info topic.");
    is_ready_ = true;
  }

  geometry_msgs::Pose transformStamped2pose(geometry_msgs::TransformStamped t) {
    geometry_msgs::Pose camera_link_pose;
    camera_link_pose.position.x = t.transform.translation.x;
    camera_link_pose.position.y = t.transform.translation.y;
    camera_link_pose.position.z = t.transform.translation.z;
    camera_link_pose.orientation.x = t.transform.rotation.x;
    camera_link_pose.orientation.y = t.transform.rotation.y;
    camera_link_pose.orientation.z = t.transform.rotation.z;
    camera_link_pose.orientation.w = t.transform.rotation.w;
    return camera_link_pose;
  }

  void DetectDarknetOpencv::rgb_depth_img_cb(const sensor_msgs::ImageConstPtr& rgb_msg_ptr,
					     const sensor_msgs::ImageConstPtr& depth_msg_ptr) {
    try {
      DetectData dd;
      dd.header = rgb_msg_ptr->header;
      dd.transformStamped = tfBuffer.lookupTransform(map_frame_name_,
						     camera_link_frame_name_,
						     rgb_msg_ptr->header.stamp,
						     ros::Duration(1.0));
      dd.rgb_msg_ptr = rgb_msg_ptr;
      dd.depth_msg_ptr = depth_msg_ptr;

      temp_dd_ = std::make_shared<DetectData>(dd);

      if (debug_) {
	int MAX = 100;
	
	if (fps_count_ == 0) {
	  fps_time_ = std::chrono::high_resolution_clock::now();
	}	  
	if (fps_count_ == MAX) {
	  auto now = std::chrono::high_resolution_clock::now();
	  auto diff = ((double)(now - fps_time_).count())/1000000000;
	  ROS_INFO("fps %.2f (%.2f, %d)", fps_count_ / diff, diff, fps_count_);
	  fps_count_ = 0;
	  fps_time_ = std::chrono::high_resolution_clock::now();
	}
	
	if (detect_count_ == MAX) {
	  ROS_INFO("detect %.2f fps %d", detect_count_ / detect_time_, detect_count_);
	  detect_time_ = 0;
	  detect_count_ = 0;
	  
	  ROS_INFO("depth %.2f fps %d", depth_count_ / depth_time_, depth_count_);
	  depth_time_ = 0;
	  depth_count_ = 0;
	}
      }
    } catch (std::exception &e) {
      ROS_INFO("tf2 error: %s", e.what());
    }
  }

  
  void DetectDarknetOpencv::fps_loop_cb(const ros::TimerEvent& event) {
    if (temp_dd_ == nullptr) {
      return;
    }
    if (queue_ready_.size() == queue_size_) {
      return;
    }
    fps_count_++;
    if (parallel_) {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_ready_.push(*temp_dd_);
      temp_dd_ = nullptr;
    } else {
      DetectData dd = *temp_dd_;
      process_detect(dd);
      process_depth(dd);
      detected_boxes_pub_.publish(dd.result);
      temp_dd_ = nullptr;
    }
  }



  void DetectDarknetOpencv::detect_loop_cb(const ros::TimerEvent& event) {
    if (queue_ready_.size() == 0) {
      return;
    }
    DetectData dd = queue_ready_.front();
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_ready_.pop();
    }
    
    auto start = std::chrono::high_resolution_clock::now();    
    process_detect(dd);
    auto end = std::chrono::high_resolution_clock::now();
    detect_time_ += ((double)(end - start).count()) / 1000000000;
    detect_count_++;

    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_detect_.push(dd);
  }
  
  void DetectDarknetOpencv::depth_loop_cb(const ros::TimerEvent& event) {
    if (queue_detect_.size() == 0) {
      return;
    }
    DetectData dd = queue_detect_.front();
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_detect_.pop();
    }

    auto start = std::chrono::high_resolution_clock::now();    
    process_depth(dd);
    detected_boxes_pub_.publish(dd.result);
    auto end = std::chrono::high_resolution_clock::now();
    depth_time_ += ((double)(end - start).count()) / 1000000000;
    depth_count_++;
  }

  void DetectDarknetOpencv::process_detect(DetectData &dd) {
    auto cv_rgb_ptr = cv_bridge::toCvShare(dd.rgb_msg_ptr, sensor_msgs::image_encodings::BGR8);
    const cv::Mat& img = cv_rgb_ptr->image;

    static std_msgs::ColorRGBA red;
    red.r = 1.0;
      
    std::vector<int> classIds;
    std::vector<float> scores;
    std::vector<cv::Rect> boxes;
    model_->detect(img, classIds, scores, boxes, 0.6, 0.4);

    track_people_py::TrackedBoxes& tbs = dd.result;
    tbs.header = dd.header;
    tbs.header.frame_id = map_frame_name_;
    tbs.camera_id = camera_id_;
    //tbs.pose = dd.getPose();
    for (int i = 0; i < classIds.size(); i++) {
      auto classId = classIds[i];
      auto score = scores[i];
      auto box = boxes[i];
      
      if (classId != 0 ||
	  score < detection_threshold_ ||
	  box.width < minimum_detection_size_threshold_ ||
	  box.height < minimum_detection_size_threshold_) {
	continue;
      }

      track_people_py::TrackedBox tb;
      tb.header = dd.header;
      tb.header.frame_id = map_frame_name_;
      tb.color = red;
      tb.box.Class = "person";
      tb.box.probability = score;
      tb.box.xmin = box.x;
      tb.box.ymin = box.y;
      tb.box.xmax = box.x + box.width;
      tb.box.ymax = box.y + box.height;
      tb.center3d.x = 1;
      tb.center3d.y = 1;
      tb.center3d.z = 1;
      tbs.tracked_boxes.push_back(tb);

      if (debug_) {
	rectangle(img, boxes[i], cv::Scalar(0, 255, 0), 2); // draw rectangle
      }
    }
  }

  void DetectDarknetOpencv::process_depth(DetectData &dd) {
    std::vector<track_people_py::TrackedBox>& tracked_boxes = dd.result.tracked_boxes;
    tf::Transform pose_tf;
    tf::transformMsgToTF(dd.transformStamped.transform, pose_tf);

    if (tracked_boxes.size() == 0) {
      detected_boxes_pub_.publish(dd.result);
      return;
    }

    for (auto it = tracked_boxes.begin(); it != tracked_boxes.end(); ) {
      auto pc = generatePointCloudFromDepthAndBox(dd, it->box);
      if (!pc->HasPoints()) {
	ROS_INFO("no points found");
	tracked_boxes.erase(it);
	continue;
      }
      
      auto median = getAxisXMedianOfPoints(*pc);
      
      if (median.hasNaN()) {
	ROS_INFO("median has NAN");
	tracked_boxes.erase(it);
	continue;
      }
      
      tf::Transform median_tf(tf::Quaternion(0,0,0,1), tf::Vector3(median(2),-median(0),0));
      auto map_center = (pose_tf*median_tf).getOrigin();
      
      it->center3d.x = map_center.x();
      it->center3d.y = map_center.y();
      it->center3d.z = map_center.z();
      it++;
    }
  }

  
  std::shared_ptr<open3d::geometry::PointCloud>
  DetectDarknetOpencv::generatePointCloudFromDepthAndBox(DetectData &dd, track_people_py::BoundingBox& box) {
    auto o3d_depth_ptr = std::make_shared<open3d::geometry::Image>();

    //ROS_INFO("depth_msg_ptr.use_count() %d", dd.depth_msg_ptr.use_count());
    auto cv_depth_ptr = cv_bridge::toCvShare(dd.depth_msg_ptr, sensor_msgs::image_encodings::TYPE_16UC1);
    //ROS_INFO("cv_depth_ptr.use_count() %d", cv_depth_ptr.use_count());
    auto img = cv_depth_ptr->image;
    //ROS_INFO("img %d %d", img.cols, img.rows);
    
    o3d_depth_ptr->Prepare(img.cols, img.rows, 1 /* channel */, 4 /* bytes per channel */); // convert to float image
    fill(o3d_depth_ptr->data_.begin(), o3d_depth_ptr->data_.end(), 0);

    for (int row = box.ymin; row < box.ymax; row++) {
      for (int col = box.xmin; col < box.xmax; col++) {
	auto p = o3d_depth_ptr->PointerAt<float>(col, row);
	*p = ((float)img.at<uint16_t>(row, col)) / 1000;
      }
    }
    /*
    for (int row = 0; row < img.rows; row++) {
      for (int col = 0; col < img.cols; col++) {
	if (box.ymin <= row && row <= box.ymax &&
	    box.xmin <= col && col <= box.xmax) {
	  auto p = o3d_depth_ptr->PointerAt<float>(col, row);
	  *p = ((float)img.at<uint16_t>(row, col)) / 1000;
	  
	  img.at<uint16_t>(row, col) *= 10;
	} else {
	  auto p = o3d_depth_ptr->PointerAt<float>(col, row);
	  *p = 0;
	  img.at<uint16_t>(row, col) = 0;
	}
      }
    }
    */

    if (debug_) {
      cv::imshow("Depth", img);
      cv::waitKey(100);
    }
      
    //ROS_INFO("%d %d %.2f %.2f %.2f %.2f", image_width_, image_height_, focal_length_, focal_length_, center_x_, center_y_);
    auto pinhole_camera_intrinsic = open3d::camera::PinholeCameraIntrinsic(image_width_, image_height_, focal_length_, focal_length_, center_x_, center_y_);

    return open3d::geometry::PointCloud::CreateFromDepthImage(*o3d_depth_ptr, pinhole_camera_intrinsic);
  }
  

  Eigen::Vector3d DetectDarknetOpencv::getAxisXMedianOfPoints(open3d::geometry::PointCloud &pc) {
    auto &ps = pc.points_;
    // partially sort until median, 50 percentile
    std::nth_element(ps.begin(), ps.begin()+ps.size()/2, ps.end(), [](Eigen::Vector3d &a, Eigen::Vector3d &b) { return a(0) < b(0); });
    return ps[ps.size()/2];
  }


} // namespace TrackPeopleCPP
