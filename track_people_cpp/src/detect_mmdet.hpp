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

#ifndef DETECT_MMDet_HPP_
#define DETECT_MMDet_HPP_

#include <mmdeploy/detector.hpp>

#include "abstract_detect_people.hpp"


namespace track_people_cpp
{

class DetectMMDet : public AbstractDetectPeople
{
public:
  explicit DetectMMDet(rclcpp::NodeOptions options);

private:
  void process_detect(DetectData & dd);

  std::shared_ptr<mmdeploy::Model> model_;
  std::shared_ptr<mmdeploy::Detector> detector_;

  // config parameters
  std::string detect_model_directory_;
};

}  // namespace track_people_cpp

#endif  // DETECT_MMDet_HPP_
