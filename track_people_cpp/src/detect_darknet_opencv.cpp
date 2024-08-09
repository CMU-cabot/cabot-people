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

#include <vector>

#include "detect_darknet_opencv.hpp"

namespace track_people_cpp
{
DetectDarknetOpencv::DetectDarknetOpencv(rclcpp::NodeOptions options)
: AbstractDetectPeople("detect_darknet_opencv_node", options)
{
  // declare parameters
  detect_config_filename_ = this->declare_parameter("detect_config_file", detect_config_filename_);
  detect_weight_filename_ = this->declare_parameter("detect_weight_file", detect_weight_filename_);
  // do not load class names, assume id 0 is person
  // this->declare_parameter("detect_label_file", detect_label_filename_);

  // load darknet model
  RCLCPP_INFO(this->get_logger(), "weights: %s", detect_weight_filename_.c_str());
  RCLCPP_INFO(this->get_logger(), "config : %s", detect_config_filename_.c_str());
  darknet_ = cv::dnn::readNet(detect_weight_filename_, detect_config_filename_, "Darknet");
  darknet_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
  darknet_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
  model_ = std::make_shared<cv::dnn::DetectionModel>(cv::dnn::DetectionModel(darknet_));
  model_->setInputParams(1 / 255.0, cv::Size(416, 416), cv::Scalar(), true);

  // fully prepare image by givng a dummy image
  cv::Mat dummy = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
  std::vector<int> classIds;
  std::vector<float> scores;
  std::vector<cv::Rect> boxes;
  model_->detect(dummy, classIds, scores, boxes, detection_threshold_, 0.4);
  is_model_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "Model Loaded");
}

/*
  actual process of darknet detection
 */
void DetectDarknetOpencv::process_detect(DetectData & dd)
{
  auto cv_rgb_ptr = cv_bridge::toCvShare(dd.rgb_msg_ptr, sensor_msgs::image_encodings::BGR8);
  const cv::Mat & img = cv_rgb_ptr->image;

  cv::Mat rImg;
  if (dd.rotate == 0) {
    rImg = img;
  } else {
    // rotate
    cv::rotate(img, rImg, dd.rotate - 1);
  }

  static std_msgs::msg::ColorRGBA red;
  red.r = 1.0;

  std::vector<int> classIds;
  std::vector<float> scores;
  std::vector<cv::Rect> boxes;
  model_->detect(rImg, classIds, scores, boxes, detection_threshold_, 0.4);

  track_people_msgs::msg::TrackedBoxes & tbs = dd.result;
  tbs.header = dd.header;
  tbs.header.frame_id = map_frame_name_;
  tbs.camera_id = camera_id_;
  // tbs.pose = dd.getPose();
  for (int i = 0; i < classIds.size(); i++) {
    auto classId = classIds[i];
    auto score = scores[i];
    auto box = boxes[i];

    // assume classId 0 is person
    if (classId != 0 || box.width < minimum_detection_size_threshold_ || box.height < minimum_detection_size_threshold_)
    {
      continue;
    }

    // rotate back the detected box coordinate
    if (dd.rotate > 0) {
      cv::Size s = rImg.size();
      int x, y, w, h;
      if (dd.rotate == 1) {
        x = box.y;
        y = s.width - box.x - box.width;
        w = box.height;
        h = box.width;
      } else if (dd.rotate == 2) {
        x = s.width - box.x - box.width;
        y = s.height - box.y - box.height;
        w = box.width;
        h = box.height;
      } else if (dd.rotate == 3) {
        x = s.height - box.y - box.height;
        y = box.x;
        w = box.height;
        h = box.width;
      }

      box.x = x;
      box.y = y;
      box.width = w;
      box.height = h;
    }

    track_people_msgs::msg::TrackedBox tb;
    tb.header = dd.header;
    tb.header.frame_id = map_frame_name_;
    tb.color = red;
    tb.box.class_name = "person";
    tb.box.probability = score;
    tb.box.xmin = box.x;
    tb.box.ymin = box.y;
    tb.box.xmax = box.x + box.width;
    tb.box.ymax = box.y + box.height;
    tb.center3d.x = 1;
    tb.center3d.y = 1;
    tb.center3d.z = 1;
    tbs.tracked_boxes.push_back(tb);
  }
}

}  // namespace track_people_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(track_people_cpp::DetectDarknetOpencv)
