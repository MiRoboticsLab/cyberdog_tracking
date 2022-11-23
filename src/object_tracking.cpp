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

#include "cyberdog_tracking/object_tracking.hpp"

#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <utility>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "cv_bridge/cv_bridge.h"
#include "tf2/LinearMath/Transform.h"
#include "rcutils/error_handling.h"

#include "cyberdog_common/cyberdog_log.hpp"

const int kBoundaryTh = 25;
const std::string kCailbrateParam = "/params/camera/calibration/params_intrinsic.yaml";

namespace cyberdog_tracking
{

ObjectTracking::ObjectTracking()
: rclcpp_lifecycle::LifecycleNode("object_tracking"),
  filter_ptr_(nullptr), unfound_count_(0),
  is_activate_(false)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  LOGGER_MAIN_INSTANCE("cyberdog_tracking");
  this->declare_parameter("ai_intrinsic", "./config/params_intrinsic.yaml");
}

ReturnResult ObjectTracking::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Configuring tracking. ");
  Initialize();
  LoadCameraParam();
  return ReturnResult::SUCCESS;
}

ReturnResult ObjectTracking::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Activating tracking. ");
  is_activate_ = true;
  // Create process thread
  handler_thread_ = std::make_shared<std::thread>(&ObjectTracking::HandlerThread, this);
  pose_pub_->on_activate();
  status_pub_->on_activate();
  return ReturnResult::SUCCESS;
}

ReturnResult ObjectTracking::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Deactivating tracking. ");
  is_activate_ = false;
  WakeThread();
  if (handler_thread_->joinable()) {
    handler_thread_->join();
  }
  pose_pub_->on_deactivate();
  status_pub_->on_deactivate();
  return ReturnResult::SUCCESS;
}

ReturnResult ObjectTracking::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Cleaning up tracking. ");
  handler_thread_.reset();
  pose_pub_.reset();
  status_pub_.reset();
  depth_sub_.reset();
  info_sub_.reset();
  body_sub_.reset();
  return ReturnResult::SUCCESS;
}

ReturnResult ObjectTracking::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  INFO("Shutting down tracking. ");
  return ReturnResult::SUCCESS;
}

void ObjectTracking::Initialize()
{
  CreateObject();
  CreateSub();
  CreatePub();
}

void ObjectTracking::CreateObject()
{
  // Get parameter
  this->get_parameter<std::string>("ai_intrinsic", ai_param_path_);
  ai_param_path_ += "/params_intrinsic.yaml";

  // Create object
  INFO("Create object.");
  filter_ptr_ = new DistanceFilter();
}

void ObjectTracking::CreateSub()
{
  // Subscribe body detection topic to process
  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  auto body_callback = [this](const PersonT::SharedPtr msg) {
      ProcessBody(msg);
    };
  INFO("Subscribing to body detection topic. ");
  body_sub_ = create_subscription<PersonT>("person", sub_qos, body_callback);

  // Subscribe depth image to process
  auto depth_callback = [this](const SensorImageT::SharedPtr msg) {
      ProcessDepth(msg);
    };
  INFO("Subscribing to depth image topic. ");
  depth_sub_ = create_subscription<SensorImageT>(
    "camera/aligned_depth_to_extcolor/image_raw",
    sub_qos, depth_callback);
}

void ObjectTracking::CreatePub()
{
  // Publish person position
  rclcpp::ServicesQoS pub_qos;
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  pose_pub_ = create_publisher<GeometryPoseStampedT>("tracking_pose", pub_qos);
  status_pub_ = create_publisher<TrackingStatusT>("tracking_status", pub_qos);
}

void ObjectTracking::ProcessDepth(const SensorImageT::SharedPtr msg)
{
  if (!is_activate_) {
    return;
  }

  INFO(
    "Received depth image, ts: %.9d.%.9d", msg->header.stamp.sec,
    msg->header.stamp.nanosec);

  cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(
    msg, sensor_msgs::image_encodings::TYPE_16UC1);
  StampedImage stamped_img;
  stamped_img.header = msg->header;
  stamped_img.image = depth_ptr->image.clone();
  std::unique_lock<std::mutex> lk(depth_mtx_);
  if (vec_stamped_depth_.size() > 6) {
    vec_stamped_depth_.erase(vec_stamped_depth_.begin());
  }
  vec_stamped_depth_.push_back(stamped_img);
}

float CalInterval(const BuiltinTimeT & stamp1, const BuiltinTimeT & stamp2)
{
  float interval = abs(
    static_cast<double>(stamp1.sec) - static_cast<double>(stamp2.sec) +
    static_cast<double>(static_cast<int>(stamp1.nanosec) - static_cast<int>(stamp2.nanosec)) *
    static_cast<double>(1.0e-9));
  return interval;
}

int FindNearest(const std::vector<StampedImage> & img_buffer, const StdHeaderT & header)
{
  int position = -1;
  float min_interval = 0.2f;
  for (size_t i = 0; i < img_buffer.size(); ++i) {
    float interval = CalInterval(img_buffer[i].header.stamp, header.stamp);
    if (interval < min_interval) {
      min_interval = interval;
      position = i;
    }
    std::cout << "interval: " << interval << std::endl;
  }
  return position;
}

cv::Rect Convert2CV(const sensor_msgs::msg::RegionOfInterest & roi)
{
  cv::Rect rect = cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height);
  return rect;
}

sensor_msgs::msg::RegionOfInterest Convert2ROS(const cv::Rect & rect)
{
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = rect.x;
  roi.y_offset = rect.y;
  roi.width = rect.width;
  roi.height = rect.height;
  return roi;
}

void ObjectTracking::ProcessBody(const PersonT::SharedPtr msg)
{
  if (!is_activate_) {
    return;
  }

  INFO(
    "Received detection result, ts %.9d.%.9d",
    msg->header.stamp.sec, msg->header.stamp.nanosec);
  INFO("Received body num %d", msg->body_info.count);

  StampedBbox tracked;
  if (0 != msg->track_res.roi.width && 0 != msg->track_res.roi.height) {
    cv::Rect bbox = Convert2CV(msg->track_res.roi);
    tracked.header = msg->header;
    tracked.vecInfo.push_back(bbox);
    std::unique_lock<std::mutex> lk(handler_.mtx);
    handler_.vecFrame.clear();
    handler_.vecFrame.push_back(tracked);
    INFO("Tracking completed, activate cloud handler to cal pose. ");
    handler_.cond.notify_one();
  }
}

void ObjectTracking::HandlerThread()
{
  while (rclcpp::ok()) {
    StampedBbox bodies;
    {
      std::unique_lock<std::mutex> lk(handler_.mtx);
      handler_.cond.wait(lk, [this] {return !handler_.vecFrame.empty();});
      INFO("Cloud handler thread is active. ");
      bodies = handler_.vecFrame[0];
      handler_.vecFrame.clear();
    }
    if (!is_activate_) {
      return;
    }

    // Get pose of tracked bbox and pub
    PubPose(bodies.header, bodies.vecInfo[0]);
  }
}

GeometryPoseT Convert2ROS(const cv::Point3f & from)
{
  GeometryPoseT to;
  to.position.x = from.x;
  to.position.y = from.y;
  to.position.z = from.z;
  return to;
}

GeometryPointT Dis2Position(
  float & distance, const std::vector<float> & cam_param,
  const cv::Vec2d & center)
{
  GeometryPointT position;
  position.x = distance;
  position.y = -distance *
    (center[0] - cam_param[2]) / cam_param[0];
  position.z = -distance *
    (center[1] - cam_param[3]) / cam_param[1];
  return position;
}

cv::Point3f Convert2CV(const GeometryPointT & point)
{
  cv::Point3f cv_point = cv::Point3f(point.x, point.y, point.z);
  return cv_point;
}

void ObjectTracking::PubStatus(const uint8_t & status)
{
  TrackingStatusT TrackingStatusT;
  TrackingStatusT.status = status;
  status_pub_->publish(TrackingStatusT);
}

void ObjectTracking::PubPose(const StdHeaderT & header, const cv::Rect & tracked)
{
  INFO("To find depth and get pose according to stamped bbox. ");

  // Get body position
  GeometryPoseStampedT pose_pub;
  pose_pub.header.stamp = header.stamp;
  pose_pub.header.frame_id = "camera_link";
  cv::Vec2d body_center = cv::Vec2d(
    tracked.x + tracked.width / 2.0,
    tracked.y + tracked.height / 2.0);

  if (filter_ptr_->initialized_ && unfound_count_ < 10) {
    // Get prediction with pre frame
    float delta = CalInterval(pose_pub.header.stamp, last_stamp_);
    cv::Point3f pose_predict = filter_ptr_->Predict(delta);
    pose_pub.pose = Convert2ROS(pose_predict);
  }

  // Get measurement value and correct
  float distance = GetDistance(header, tracked);
  GeometryPoseStampedT pose;
  pose.header = pose_pub.header;
  pose.pose.position = Dis2Position(distance, ai_intrinsics_, body_center);
  pose.pose.orientation.w = 1.0;
  if (0.0 != distance) {
    INFO("Get pose success, correct kf. ");
    cv::Point3f posePost = filter_ptr_->Correct(Convert2CV(pose.pose.position));
    pose_pub.pose = Convert2ROS(posePost);
  }

  // Publish position only valid
  if (filter_ptr_->initialized_ && unfound_count_ < 10) {
    pose_pub_->publish(pose_pub);
    last_stamp_ = pose_pub.header.stamp;
    INFO(
      "Pose pub: %.5f, %.5f, %.5f",
      pose_pub.pose.position.x,
      pose_pub.pose.position.y,
      pose_pub.pose.position.z);

    double dis = sqrt(pow(pose_pub.pose.position.x, 2) + pow(pose_pub.pose.position.y, 2));
    INFO("Straight line distance from person to dog: %f", dis);
    if (dis > 3.0) {
      INFO("Status OBJECT_FAR.");
      PubStatus(TrackingStatusT::OBJECT_FAR);
    } else if (dis < 0.8) {
      INFO("Status OBJECT_NEAR.");
      PubStatus(TrackingStatusT::OBJECT_NEAR);
    }
    std::cout << "###posepub: " << pose_pub.pose.position.x << std::endl;
  } else {
    std::cout << "###posepub: null " << std::endl;
  }
}

float ObjectTracking::GetDistance(const StdHeaderT & header, const cv::Rect & tracked)
{
  // Find depth image according to timestamp
  cv::Mat depth_image;
  {
    std::unique_lock<std::mutex> lk(depth_mtx_);
    int position = FindNearest(vec_stamped_depth_, header);
    if (position >= 0) {
      depth_image = vec_stamped_depth_[position].image.clone();
      INFO(
        "Find depth image ts:  %.9d.%.9d",
        vec_stamped_depth_[position].header.stamp.sec,
        vec_stamped_depth_[position].header.stamp.nanosec);
    } else {
      ERROR("Cannot find depth image, cannot calculate pose. ");
    }
  }

  // Align depth to ai and get distance
  float distance = 0.f;
  cv::Mat ai_depth = depth_image.clone();
  if (!depth_image.empty()) {
    if (tracked.x > kBoundaryTh && tracked.x + tracked.width < (img_width_ - kBoundaryTh)) {
      INFO("Get distance according to cloud point. ");
      double start = static_cast<double>(cv::getTickCount());
      double time = (static_cast<double>(cv::getTickCount()) - start) / cv::getTickFrequency();
      DEBUG("Cloud handler cost: %f ms", time * 1000);
    } else {
      INFO("Status OBJECT_EDGE.");
      PubStatus(TrackingStatusT::OBJECT_EDGE);
    }

    // Get person position and publish
    distance = GetDistance(ai_depth.clone(), tracked);
    if (0.0 != distance) {
      unfound_count_ = 0;
    } else {
      unfound_count_++;
    }
    std::cout << "###discal: " << distance << std::endl;
  } else {
    unfound_count_++;
    std::cout << "###discal: null " << std::endl;
  }

  if (unfound_count_ >= 10) {
    filter_ptr_->initialized_ = false;
  }

  return distance;
}

float ObjectTracking::GetDistance(const cv::Mat & image, const cv::Rect2d & body_tracked)
{
  float distance = 0.f;   // unit m

  if (!image.empty()) {
    std::map<int, int> map_depth;
    for (size_t i = body_tracked.x; i < body_tracked.x + body_tracked.width; ++i) {
      for (size_t j = body_tracked.y; j < body_tracked.y + body_tracked.height; ++j) {
        if (0.0 != image.at<ushort>(j, i)) {
          int val = floor(image.at<ushort>(j, i) / 100.0);
          std::map<int, int>::iterator iter = map_depth.find(val);
          if (iter != map_depth.end()) {
            iter->second++;
          } else {
            map_depth.insert(std::pair<int, int>(val, 1));
          }
        }
      }
    }

    int sum_count = 0;
    for (std::map<int, int>::iterator it = map_depth.begin(); it != map_depth.end(); ++it) {
      // std::cout << it->first << "-" << it->second << "; ";
      sum_count += it->second;
    }
    // std::cout << std::endl;
    INFO("===sum_count: %d", sum_count);

    int dis_sum = 0;
    int dis_count = 0;
    bool is_find = false;

    int last_value = 0;
    if (!map_depth.empty()) {
      last_value = map_depth.begin()->first;
    }
    if (sum_count > 100) {
      for (std::map<int, int>::iterator it = map_depth.begin(); it != map_depth.end() && !is_find;
        ++it)
      {
        if ((static_cast<double>(it->second) / sum_count) > 0.02 && (it->first - last_value) < 2) {
          dis_count += it->second;
          dis_sum += it->first / 10.0 * it->second;
        } else {
          if (dis_count / static_cast<double>(sum_count) > 0.2) {
            is_find = true;
            distance = dis_sum / static_cast<double>(dis_count);
          }
          dis_sum = 0;
          dis_count = 0;
        }
        last_value = it->first;
      }
    } else {
      ERROR("Cloud point < 100 . ");
    }
  }

  return distance;
}

void ObjectTracking::WakeThread()
{
  StampedBbox tracked;
  std::unique_lock<std::mutex> lk(handler_.mtx);
  handler_.vecFrame.clear();
  handler_.vecFrame.push_back(tracked);
  handler_.cond.notify_one();
}

void ObjectTracking::LoadCameraParam()
{
  YAML::Node param;
  try {
    param = YAML::LoadFile(kCailbrateParam);
    ai_intrinsics_ = param["cam3"]["intrinsics"].as<std::vector<float>>();
    if (ai_intrinsics_.size() != 4) {
      ERROR("Non-pinhole model, use default param. ");
      param = YAML::LoadFile(ai_param_path_);
      ai_intrinsics_.clear();
      ai_intrinsics_ = param["cam3"]["intrinsics"].as<std::vector<float>>();
    }
  } catch (...) {
    ERROR("Load param fail, use default param. ");
    param = YAML::LoadFile(ai_param_path_);
    ai_intrinsics_ = param["cam3"]["intrinsics"].as<std::vector<float>>();
  }

  std::vector<int> img_size = param["cam3"]["resolution"].as<std::vector<int>>();
  img_width_ = img_size[0];
  img_height_ = img_size[1];
}

ObjectTracking::~ObjectTracking()
{
  if (filter_ptr_ != nullptr) {
    delete filter_ptr_;
  }

  WakeThread();
  if (handler_thread_->joinable()) {
    handler_thread_->join();
  }
}

}  // namespace cyberdog_tracking
