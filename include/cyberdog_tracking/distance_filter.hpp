// Copyright (c) 2021 Xiaomi Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CYBERDOG_TRACKING__DISTANCE_FILTER_HPP_
#define CYBERDOG_TRACKING__DISTANCE_FILTER_HPP_

#include "opencv2/video/tracking.hpp"

namespace cyberdog_tracking
{

class DistanceFilter
{
public:
  DistanceFilter();
  ~DistanceFilter();

  void Init(const cv::Point3f & pose);
  cv::Point3f Predict(const float & delta);
  cv::Point3f Correct(const cv::Point3f & pose);

  bool initialized_;
  cv::KalmanFilter filter_;

private:
  void Init();
  void SetInterval(const float & delta);

  int state_num_;
  int measure_num_;

  cv::Mat state_;
  cv::Mat process_noise_;
  cv::Mat measurement_;
};

}  // namespace cyberdog_tracking

#endif  // CYBERDOG_TRACKING__DISTANCE_FILTER_HPP_
