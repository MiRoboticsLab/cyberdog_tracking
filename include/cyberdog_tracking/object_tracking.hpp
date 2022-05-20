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

#ifndef CYBERDOG_TRACKING__OBJECT_TRACKING_HPP_
#define CYBERDOG_TRACKING__OBJECT_TRACKING_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "cyberdog_tracking/transform.hpp"
#include "cyberdog_tracking/common_type.hpp"
#include "cyberdog_tracking/distance_filter.hpp"

namespace cyberdog_tracking
{

class ObjectTracking : public rclcpp::Node
{
public:
  ObjectTracking();
  virtual ~ObjectTracking();

  int SuspendTracking();

private:
  void Initialize();
  void CreateObject();
  void CreateSub();
  void CreatePub();
  void CreateSrv();

  void ProcessDepth(const sensor_msgs::msg::Image::SharedPtr msg, rclcpp::Logger logger);
  void ProcessInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg, rclcpp::Logger logger);
  void ProcessBody(const BodyInfo::SharedPtr msg, rclcpp::Logger logger);
  void ProcessFace(const FaceInfo::SharedPtr msg, rclcpp::Logger logger);

  bool CallService(
    rclcpp::Client<CameraService>::SharedPtr & client, const uint8_t & cmd,
    const std::string & args);
  void TrackingCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<BodyRegion::Request> req,
    std::shared_ptr<BodyRegion::Response> res);
  void ModeCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<NavMode::Request> req,
    std::shared_ptr<NavMode::Response> res);

  void HandlerThread();

  void PubStatus(const uint8_t & status);
  void PubPose(const std_msgs::msg::Header & header, const PersonInfo & tracked);
  void PubRect(const std_msgs::msg::Header & header, const PersonInfo & tracked);

  float GetDistance(const cv::Mat & image, const cv::Rect2d & body_tracked);
  int GetMatchFace(const StampedBbox & detections, const StampedBbox & stamped_face);
  int GetMatchBody(const StampedBbox & detections, const cv::Rect & body);
  int ProcessObject(const std_msgs::msg::Header, const cv::Rect & bbox);

  bool GetExtrinsicsParam(cv::Mat & rot_mat, cv::Mat & trans_mat);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<BodyInfo>::SharedPtr body_sub_;
  rclcpp::Subscription<FaceInfo>::SharedPtr face_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<BodyInfo>::SharedPtr rect_pub_;
  rclcpp::Publisher<TrackingStatus>::SharedPtr status_pub_;

  rclcpp::Service<NavMode>::SharedPtr mode_service_;
  rclcpp::Service<BodyRegion>::SharedPtr tracking_service_;
  rclcpp::Client<CameraService>::SharedPtr body_client_;
  rclcpp::Client<CameraService>::SharedPtr face_client_;
  rclcpp::Client<CameraService>::SharedPtr reid_client_;

  sensor_msgs::msg::CameraInfo camera_info_;
  builtin_interfaces::msg::Time last_stamp_;

  Transform * trans_ptr_;
  DistanceFilter * filter_ptr_;

  HandlerStruct handler_;
  std::shared_ptr<std::thread> handler_thread_;

  std::mutex depth_mtx_;
  std::vector<StampedImage> vec_stamped_depth_;

  std::mutex detections_mut_;
  StampedBbox curr_detections_;

  std_msgs::msg::Header start_time_;

  bool stereo_mode_;
  float row_scale_;
  float col_scale_;
  int logger_level_;
  std::string param_path_;

  int uncorrect_count_;
  int unmatch_count_;

  bool tracking_;
  bool deactivate_;
  bool face_mode_;

  uint8_t curr_status_;
};

}  // namespace cyberdog_tracking

#endif  // CYBERDOG_TRACKING__OBJECT_TRACKING_HPP_
