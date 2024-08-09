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

#include "detect_mmdet.hpp"

namespace track_people_cpp
{
DetectMMDet::DetectMMDet(rclcpp::NodeOptions options)
: AbstractDetectPeople("detect_mmdet_node", options)
{
  // declare parameters
  detect_model_directory_ = this->declare_parameter("detect_model_dir", detect_model_directory_);

  // load MMDetection model
  RCLCPP_INFO(this->get_logger(), "model : %s", detect_model_directory_.c_str());
  model_ = std::make_shared<mmdeploy::Model>(detect_model_directory_);
  detector_ = std::make_shared<mmdeploy::Detector>(*model_, mmdeploy::Device{"cuda", 0});
  is_model_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "Model Loaded");
}

/*
  actual process of darknet detection
 */
void DetectMMDet::process_detect(DetectData & dd)
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

  mmdeploy::Detector::Result dets = detector_->Apply(rImg);

  track_people_msgs::msg::TrackedBoxes & tbs = dd.result;
  tbs.header = dd.header;
  tbs.header.frame_id = map_frame_name_;
  tbs.camera_id = camera_id_;
  // tbs.pose = dd.getPose();
  for (int i = 0; i < dets.size(); i++) {
    auto classId = dets[i].label_id;
    auto score = dets[i].score;
    auto box = dets[i].bbox;

    // assume classId 0 is person
    if ((classId != 0) || (score < detection_threshold_) 
      || (box.right-box.left < minimum_detection_size_threshold_) || (box.bottom-box.top < minimum_detection_size_threshold_))
    {
      continue;
    }

    // rotate back the detected box coordinate
    if (dd.rotate > 0) {
      cv::Size s = rImg.size();
      int left, top, right, bottom;
      if (dd.rotate == 1) {
        left = box.top;
        top = s.width - box.right;
        right = left + (box.bottom - box.top);
        bottom = top + (box.right - box.left);
      } else if (dd.rotate == 2) {
        left = s.width - box.right;
        top = s.height - box.bottom;
        right = left + (box.right - box.left);
        bottom = top + (box.bottom - box.top);
      } else if (dd.rotate == 3) {
        left = s.height - box.bottom;
        top = box.left;
        right = left + (box.bottom - box.top);
        bottom = top + (box.right - box.left);
      }

      box.left = left;
      box.top = top;
      box.right = right;
      box.bottom = bottom;
    }

    track_people_msgs::msg::TrackedBox tb;
    tb.header = dd.header;
    tb.header.frame_id = map_frame_name_;
    tb.color = red;
    tb.box.class_name = "person";
    tb.box.probability = score;
    tb.box.xmin = box.left;
    tb.box.ymin = box.top;
    tb.box.xmax = box.right;
    tb.box.ymax = box.bottom;
    tb.center3d.x = 1;
    tb.center3d.y = 1;
    tb.center3d.z = 1;
    tbs.tracked_boxes.push_back(tb);
  }
}

}  // namespace track_people_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(track_people_cpp::DetectMMDet)
