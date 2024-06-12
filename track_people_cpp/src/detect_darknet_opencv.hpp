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

#ifndef DETECT_DARKNET_OPENCV_HPP_
#define DETECT_DARKNET_OPENCV_HPP_

#include <opencv2/dnn.hpp>

#include "abstract_detect_people.hpp"


namespace track_people_cpp
{

class DetectDarknetOpencv : public AbstractDetectPeople
{
public:
  explicit DetectDarknetOpencv(rclcpp::NodeOptions options);

private:
  void process_detect(DetectData & dd);

  cv::dnn::Net darknet_;

  std::shared_ptr<cv::dnn::DetectionModel> model_;

  // config parameters
  std::string detect_config_filename_;
  std::string detect_weight_filename_;
  std::string detect_label_filename_;
};

}  // namespace track_people_cpp

#endif  // DETECT_DARKNET_OPENCV_HPP_
