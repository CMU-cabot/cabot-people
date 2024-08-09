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

#include "detect_mmdet_seg.hpp"

namespace track_people_cpp
{
DetectMMDetSeg::DetectMMDetSeg(rclcpp::NodeOptions options)
: AbstractDetectPeople("detect_mmdet_seg_node", options)
{
  // declare parameters
  model_input_width_ = this->declare_parameter("model_input_width", model_input_width_);
  model_input_height_ = this->declare_parameter("model_input_height", model_input_height_);
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
void DetectMMDetSeg::process_detect(DetectData & dd)
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

  // resize input image to avoid slow postprocessing issue
  // https://github.com/open-mmlab/mmdeploy/issues/2512#issuecomment-1774623268
  double resize_width_ratio = model_input_width_ / double(rImg.cols);
  double resize_height_ratio = model_input_height_ / double(rImg.rows);
  cv::Mat rResizeImage;
  cv::resize(rImg, rResizeImage, cv::Size(model_input_width_, model_input_height_));

  static std_msgs::msg::ColorRGBA red;
  red.r = 1.0;

  mmdeploy::Detector::Result dets = detector_->Apply(rResizeImage);

  track_people_msgs::msg::TrackedBoxes & tbs = dd.result;
  std::vector<cv::Mat> & masks = dd.masks;
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

    // create mask image from mask image cropped by bbox
    cv::Mat cropped_mask(dets[i].mask->height, dets[i].mask->width, CV_8UC1, dets[i].mask->data);
    cv::Mat mask = cv::Mat::zeros(cv::Size(model_input_width_, model_input_height_), CV_8UC1);
    int mask_left = (box.left+dets[i].mask->width<=model_input_width_) ? box.left : model_input_width_-dets[i].mask->width;
    int mask_top = (box.top+dets[i].mask->height<=model_input_height_) ? box.top : model_input_height_-dets[i].mask->height;
    cropped_mask.copyTo(mask(cv::Rect(box.left, box.top, dets[i].mask->width, dets[i].mask->height)));

    // resize detected box to original image size
    box.left = int(box.left / resize_width_ratio);
    box.top = int(box.top / resize_height_ratio);
    box.right = int(box.right / resize_width_ratio);
    box.bottom = int(box.bottom / resize_height_ratio);

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

    // resize mask to original image size
    cv::resize(mask, mask, rImg.size(), cv::INTER_NEAREST);
    // rotate back mask
    if (dd.rotate == 1) {
      cv::rotate(mask, mask, cv::ROTATE_90_COUNTERCLOCKWISE);
    } else if (dd.rotate == 2) {
      cv::rotate(mask, mask, cv::ROTATE_180);
    } else if (dd.rotate == 3) {
      cv::rotate(mask, mask, cv::ROTATE_90_CLOCKWISE);
    }
    masks.push_back(mask);
  }
}

}  // namespace track_people_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(track_people_cpp::DetectMMDetSeg)
