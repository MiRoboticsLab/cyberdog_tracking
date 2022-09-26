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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "cyberdog_tracking/transform.hpp"
#include "cyberdog_tracking/common_type.hpp"
#include "cyberdog_tracking/distance_filter.hpp"

namespace cyberdog_tracking
{
using ReturnResult = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ObjectTracking : public rclcpp_lifecycle::LifecycleNode
{
public:
  ObjectTracking();
  virtual ~ObjectTracking();

protected:
  ReturnResult on_configure(const rclcpp_lifecycle::State & state) override;
  ReturnResult on_activate(const rclcpp_lifecycle::State & state) override;
  ReturnResult on_deactivate(const rclcpp_lifecycle::State & state) override;
  ReturnResult on_cleanup(const rclcpp_lifecycle::State & state) override;
  ReturnResult on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void Initialize();
  void CreateObject();
  void CreateSub();
  void CreatePub();

  void ProcessDepth(const SensorImageT::SharedPtr msg, rclcpp::Logger logger);
  void ProcessInfo(const SensorCameraInfoT::SharedPtr msg, rclcpp::Logger logger);
  void ProcessBody(const PersonT::SharedPtr msg, rclcpp::Logger logger);

  void HandlerThread();

  void PubStatus(const uint8_t & status);
  void PubPose(const StdHeaderT & header, const PersonInfo & tracked);

  float GetDistance(const StdHeaderT & header, const PersonInfo & tracked);
  float GetDistance(const cv::Mat & image, const cv::Rect2d & body_tracked);
  bool GetExtrinsicsParam(cv::Mat & rot_mat, cv::Mat & trans_mat);

private:
  rclcpp::Subscription<SensorImageT>::SharedPtr depth_sub_;
  rclcpp::Subscription<SensorCameraInfoT>::SharedPtr info_sub_;
  rclcpp::Subscription<PersonT>::SharedPtr body_sub_;

  rclcpp_lifecycle::LifecyclePublisher<GeometryPoseStampedT>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<TrackingStatusT>::SharedPtr status_pub_;

  SensorCameraInfoT camera_info_;
  BuiltinTimeT last_stamp_;

  Transform * trans_ptr_;
  DistanceFilter * filter_ptr_;

  HandlerStruct handler_;
  std::shared_ptr<std::thread> handler_thread_;

  std::mutex depth_mtx_;
  std::vector<StampedImage> vec_stamped_depth_;

  bool stereo_mode_;
  float row_scale_;
  float col_scale_;
  int logger_level_;
  std::string param_path_;

  int unfound_count_;
  bool is_activate_;
};

}  // namespace cyberdog_tracking

#endif  // CYBERDOG_TRACKING__OBJECT_TRACKING_HPP_
