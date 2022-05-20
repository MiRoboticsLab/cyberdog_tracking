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

#ifndef CYBERDOG_TRACKING__COMMON_TYPE_HPP_
#define CYBERDOG_TRACKING__COMMON_TYPE_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "opencv2/opencv.hpp"

#include "std_msgs/msg/header.hpp"

#include "protocol/msg/body.hpp"
#include "protocol/msg/body_info.hpp"
#include "protocol/msg/face.hpp"
#include "protocol/msg/face_info.hpp"
#include "protocol/srv/camera_service.hpp"
#include "protocol/srv/body_region.hpp"
#include "protocol/srv/nav_mode.hpp"
#include "protocol/msg/tracking_status.hpp"

namespace cyberdog_tracking
{

using CameraService = protocol::srv::CameraService;
using BodyRegion = protocol::srv::BodyRegion;
using Body = protocol::msg::Body;
using BodyInfo = protocol::msg::BodyInfo;
using FaceInfo = protocol::msg::FaceInfo;
using NavMode = protocol::srv::NavMode;
using TrackingStatus = protocol::msg::TrackingStatus;

struct PersonInfo
{
  cv::Rect bbox;
  std::string id;
  PersonInfo()
  : id("")
  {
  }
  explicit PersonInfo(const cv::Rect & bbox_, const std::string & id_ = "")
  : bbox(bbox_), id(id_)
  {
  }
};

struct StampedImage
{
  std_msgs::msg::Header header;
  cv::Mat image;
};

struct StampedBbox
{
  std_msgs::msg::Header header;
  std::vector<PersonInfo> vecInfo;
};

struct StampedPose
{
  std_msgs::msg::Header header;
  cv::Vec3d pose;
};

struct FaceStruct
{
  std::mutex mtx;
  std::condition_variable cond;
  StampedBbox stampedFece;
};

struct HandlerStruct
{
  std::mutex mtx;
  std::condition_variable cond;
  std::vector<StampedBbox> vecFrame;
};

}  // namespace cyberdog_tracking

#endif  // CYBERDOG_TRACKING__COMMON_TYPE_HPP_
