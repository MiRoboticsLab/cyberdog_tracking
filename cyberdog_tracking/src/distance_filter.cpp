// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include "cyberdog_tracking/distance_filter.hpp"

namespace cyberdog_tracking
{

cv::Point3f Convert(const cv::Mat & from)
{
  cv::Point3f to = cv::Point3f(from.at<float>(0), from.at<float>(1), from.at<float>(2));
  return to;
}

DistanceFilter::DistanceFilter()
: initialized_(false), state_num_(6), measure_num_(3)
{
  Init();
}

void DistanceFilter::Init(const cv::Point3f & pose)
{
  filter_.statePost.at<float>(0) = pose.x;
  filter_.statePost.at<float>(1) = pose.y;
  filter_.statePost.at<float>(2) = pose.z;
  initialized_ = true;
}

cv::Point3f DistanceFilter::Predict(const float & delta)
{
  SetInterval(delta);
  cv::Mat prediction = filter_.predict();
  cv::Point3f predict_pose = Convert(prediction);
  return predict_pose;
}

cv::Point3f DistanceFilter::Correct(const cv::Point3f & pose)
{
  if (!initialized_) {
    Init(pose);
    return pose;
  }

  measurement_.at<float>(0) = pose.x;
  measurement_.at<float>(1) = pose.y;
  measurement_.at<float>(2) = pose.z;
  cv::Mat pose_post = filter_.correct(measurement_);
  cv::Point3f pose_corrected = Convert(pose_post);
  return pose_corrected;
}

void DistanceFilter::Init()
{
  filter_ = cv::KalmanFilter(state_num_, measure_num_, 0);
  state_.create(state_num_, 1, CV_32FC1);
  process_noise_.create(state_num_, 1, CV_32F);
  measurement_ = cv::Mat::zeros(measure_num_, 1, CV_32F);

  randn(state_, cv::Scalar::all(0), cv::Scalar::all(0.1));
  filter_.transitionMatrix = (cv::Mat_<float>(6, 6) << \
    1, 0, 0, 1, 0, 0, \
    0, 1, 0, 0, 1, 0, \
    0, 0, 1, 0, 0, 1, \
    0, 0, 0, 1, 0, 0, \
    0, 0, 0, 0, 1, 0, \
    0, 0, 0, 0, 0, 1);

  filter_.measurementMatrix = (cv::Mat_<float>(3, 6) << \
    1, 0, 0, 0, 0, 0, \
    0, 1, 0, 0, 0, 0, \
    0, 0, 1, 0, 0, 0);

  setIdentity(filter_.processNoiseCov, cv::Scalar::all(1e-5));
  setIdentity(filter_.measurementNoiseCov, cv::Scalar::all(1e-4));
  setIdentity(filter_.errorCovPost, cv::Scalar::all(1));
}

void DistanceFilter::SetInterval(const float & delta)
{
  filter_.transitionMatrix.at<float>(0, 3) = delta;
  filter_.transitionMatrix.at<float>(1, 4) = delta;
  filter_.transitionMatrix.at<float>(2, 5) = delta;
}

DistanceFilter::~DistanceFilter()
{
}

}  // namespace cyberdog_tracking
