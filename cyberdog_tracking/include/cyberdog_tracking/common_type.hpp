// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "protocol/msg/body.hpp"
#include "protocol/msg/body_info.hpp"
#include "protocol/msg/person.hpp"
#include "protocol/msg/tracking_status.hpp"
#include "protocol/srv/camera_service.hpp"
#include "protocol/srv/body_region.hpp"
#include "protocol/srv/nav_mode.hpp"

namespace cyberdog_tracking
{

using StdHeaderT = std_msgs::msg::Header;
using SensorImageT = sensor_msgs::msg::Image;
using SensorCameraInfoT = sensor_msgs::msg::CameraInfo;
using GeometryPointT = geometry_msgs::msg::Point;
using GeometryPoseT = geometry_msgs::msg::Pose;
using GeometryPoseStampedT = geometry_msgs::msg::PoseStamped;
using BuiltinTimeT = builtin_interfaces::msg::Time;

using BodyT = protocol::msg::Body;
using BodyInfoT = protocol::msg::BodyInfo;
using PersonT = protocol::msg::Person;
using TrackingStatusT = protocol::msg::TrackingStatus;
using BodyRegionT = protocol::srv::BodyRegion;
using CameraServiceT = protocol::srv::CameraService;

struct StampedImage
{
  StdHeaderT header;
  cv::Mat image;
};

struct StampedBbox
{
  StdHeaderT header;
  std::vector<cv::Rect> vecInfo;
};

struct HandlerStruct
{
  std::mutex mtx;
  std::condition_variable cond;
  std::vector<StampedBbox> vecFrame;
};

}  // namespace cyberdog_tracking

#endif  // CYBERDOG_TRACKING__COMMON_TYPE_HPP_
