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

namespace cyberdog_tracking
{

ObjectTracking::ObjectTracking()
: Node("object_tracking"),
  trans_ptr_(nullptr), filter_ptr_(nullptr),
  uncorrect_count_(0), unfound_count_(0)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  Initialize();
}

void ObjectTracking::Initialize()
{
  CreateObject();
  CreateSub();
  CreatePub();

  // Create process thread
  handler_thread_ = std::make_shared<std::thread>(&ObjectTracking::HandlerThread, this);

  // Set logger level
  auto ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), logger_level_);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(this->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

void ObjectTracking::CreateObject()
{
  // Declare parameter
  this->declare_parameter("logger_level", 20);
  this->declare_parameter("stereo_mode", false);
  this->declare_parameter("remap_rows_scale", 1.0);
  this->declare_parameter("remap_cols_scale", 1.0);
  this->declare_parameter("camera_ai_param", "./config/camera_AI.yaml");

  // Get parameter
  this->get_parameter<int>("logger_level", logger_level_);
  this->get_parameter<bool>("stereo_mode", stereo_mode_);
  this->get_parameter<float>("remap_rows_scale", row_scale_);
  this->get_parameter<float>("remap_cols_scale", col_scale_);
  param_path_ = "/params/camera";
  if (access(param_path_.c_str(), 0) != 0) {
    RCLCPP_WARN(this->get_logger(), "The factory preset calibration file does not exist. ");
    this->get_parameter<std::string>("camera_ai_param", param_path_);
  }
  RCLCPP_INFO(this->get_logger(), "Current calibration file path: %s", param_path_.c_str());

  // Get extrinsics parameters through tf
  RCLCPP_INFO(this->get_logger(), "Get extrinsics parameters.");
  cv::Mat rot_depth_to_color, trans_depth_to_color;
  if (!GetExtrinsicsParam(rot_depth_to_color, trans_depth_to_color)) {
    RCLCPP_ERROR(this->get_logger(), "Get external parameter fail, cannot align depth to color. ");
  }

  // Create object
  RCLCPP_INFO(this->get_logger(), "Create object.");
  trans_ptr_ = new Transform(param_path_, rot_depth_to_color, trans_depth_to_color, stereo_mode_);
  filter_ptr_ = new DistanceFilter();
}

void ObjectTracking::CreateSub()
{
  // Subscribe body detection topic to process
  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  auto body_callback = [this](const PersonT::SharedPtr msg) {
      ProcessBody(msg, this->get_logger());
    };
  RCLCPP_INFO(this->get_logger(), "Subscribing to body detection topic. ");
  body_sub_ = create_subscription<PersonT>("person", sub_qos, body_callback);

  // Subscribe depth image to process
  // rclcpp::SensorDataQoS depth_qos;
  // depth_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  auto depth_callback = [this](const SensorImageT::SharedPtr msg) {
      ProcessDepth(msg, this->get_logger());
    };
  RCLCPP_INFO(this->get_logger(), "Subscribing to depth image topic. ");
  depth_sub_ = create_subscription<SensorImageT>(
    "camera/depth/image_rect_raw",
    10, depth_callback);

  // Subscribe realsense depth camera info
  rclcpp::SensorDataQoS info_qos;
  info_qos.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
  auto info_callback = [this](const SensorCameraInfoT::SharedPtr msg) {
      ProcessInfo(msg, this->get_logger());
    };
  RCLCPP_INFO(this->get_logger(), "Subscribing to depth camera info topic. ");
  info_sub_ = create_subscription<SensorCameraInfoT>(
    "camera/depth/camera_info",
    info_qos, info_callback);
}

void ObjectTracking::CreatePub()
{
  // Publish person position
  rclcpp::ServicesQoS pub_qos;
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  pose_pub_ = create_publisher<GeometryPoseStampedT>("tracking_pose", pub_qos);
  status_pub_ = create_publisher<TrackingStatusT>("tracking_status", pub_qos);
}

void ObjectTracking::ProcessDepth(
  const SensorImageT::SharedPtr msg,
  rclcpp::Logger logger)
{
  RCLCPP_INFO(
    logger, "Received depth image, ts: %.9d.%.9d", msg->header.stamp.sec,
    msg->header.stamp.nanosec);

  cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(msg);
  StampedImage stamped_img;
  stamped_img.header = msg->header;
  stamped_img.image = depth_ptr->image.clone();
  std::unique_lock<std::mutex> lk(depth_mtx_);
  if (vec_stamped_depth_.size() > 6) {
    vec_stamped_depth_.erase(vec_stamped_depth_.begin());
  }
  vec_stamped_depth_.push_back(stamped_img);
}

void ObjectTracking::ProcessInfo(
  const SensorCameraInfoT::SharedPtr msg,
  rclcpp::Logger logger)
{
  RCLCPP_INFO(
    logger, "Received depth camera info msg, ts: %.9d.%.9d", msg->header.stamp.sec,
    msg->header.stamp.nanosec);
  camera_info_ = *msg;
  info_sub_ = nullptr;
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
  std::cout << "tofind stamp: " << header.stamp.sec << ", " << header.stamp.nanosec << std::endl;
  std::cout << "depth img stamp: " << std::endl;
  for (auto & img : img_buffer) {
    std::cout << img.header.stamp.sec << ", " << img.header.stamp.nanosec << ";";
  }
  std::cout << std::endl;
  int position = -1;
  float min_interval = 0.1;
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

void ObjectTracking::ProcessBody(const PersonT::SharedPtr msg, rclcpp::Logger logger)
{
  RCLCPP_INFO(
    logger, "Received detection result, ts %.9d.%.9d", msg->header.stamp.sec,
    msg->header.stamp.nanosec);
  RCLCPP_INFO(logger, "Received body num %d", msg->body_info.count);

  StampedBbox tracked;
  std::vector<PersonInfo> bbox_det;
  for (size_t i = 0; i < msg->body_info.count; ++i) {
    cv::Rect bbox = Convert2CV(msg->body_info.infos[i].roi);
    PersonInfo info(bbox, msg->body_info.infos[i].reid);
    bbox_det.push_back(info);
    if (!msg->body_info.infos[i].reid.empty()) {
      tracked.vecInfo.push_back(info);
      tracked.header = msg->header;
      std::unique_lock<std::mutex> lk(handler_.mtx);
      handler_.vecFrame.clear();
      handler_.vecFrame.push_back(tracked);
      RCLCPP_INFO(this->get_logger(), "Tracking completed, activate cloud handler to cal pose. ");
      handler_.cond.notify_one();
    }
  }
}

void ObjectTracking::HandlerThread()
{
  while (rclcpp::ok()) {
    StampedBbox bodies;
    {
      std::unique_lock<std::mutex> lk(handler_.mtx);
      handler_.cond.wait(lk, [this] {return !handler_.vecFrame.empty();});
      RCLCPP_INFO(this->get_logger(), "Cloud handler thread is active. ");
      bodies = handler_.vecFrame[0];
      handler_.vecFrame.clear();
    }

    // Save and find tracking result according to reid
    PersonInfo person_tracked;
    for (size_t i = 0; i < bodies.vecInfo.size(); ++i) {
      if (!bodies.vecInfo[i].id.empty()) {
        person_tracked = bodies.vecInfo[i];
      }
    }

    // Get pose of tracked bbox and pub
    PubPose(bodies.header, person_tracked);
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
  float & distance, const cv::Mat & cam_param,
  const cv::Vec2d & center)
{
  GeometryPointT position;
  position.x = distance;
  position.y = -distance *
    (center[0] - cam_param.at<double>(0, 2)) / cam_param.at<double>(0, 0);
  position.z = -distance *
    (center[1] - cam_param.at<double>(1, 2)) / cam_param.at<double>(1, 1);
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

void ObjectTracking::PubPose(const StdHeaderT & header, const PersonInfo & tracked)
{
  if (tracked.id.empty()) {
    return;
  }

  // Find depth image according to timestamp
  cv::Mat depth_image;
  {
    std::unique_lock<std::mutex> lk(depth_mtx_);
    RCLCPP_DEBUG(
      this->get_logger(), "===To find depth image according to bbox ts: %.9d.%.9d",
      header.stamp.sec, header.stamp.nanosec);
    int position = FindNearest(vec_stamped_depth_, header);
    if (position >= 0) {
      depth_image = vec_stamped_depth_[position].image.clone();
      RCLCPP_INFO(
        this->get_logger(), "===Find depth image ts:  %.9d.%.9d",
        vec_stamped_depth_[position].header.stamp.sec,
        vec_stamped_depth_[position].header.stamp.nanosec);
    }
  }

  // Get body position and publish
  GeometryPoseStampedT pose_pub;
  pose_pub.header.stamp = header.stamp;
  pose_pub.header.frame_id = "camera_link";
  cv::Mat cam_param = trans_ptr_->ai_camera_param_;
  cv::Vec2d body_center = cv::Vec2d(
    tracked.bbox.x + tracked.bbox.width / 2.0,
    tracked.bbox.y + tracked.bbox.height / 2.0);
  // Kalman predict with pre frame
  if (filter_ptr_->initialized_ && unfound_count_ < 10) {
    float delta = CalInterval(pose_pub.header.stamp, last_stamp_);
    cv::Point3f posePredict = filter_ptr_->Predict(delta);
    pose_pub.pose = Convert2ROS(posePredict);
  }

  cv::Mat ai_depth;
  if (!depth_image.empty()) {
    unfound_count_ = 0;
    if (tracked.bbox.x > 25 && tracked.bbox.x + tracked.bbox.width < 615) {
      RCLCPP_INFO(this->get_logger(), "Get distance according to cloud point. ");
      double start = static_cast<double>(cv::getTickCount());
      ai_depth = trans_ptr_->DepthToAi(depth_image, camera_info_, row_scale_, col_scale_);
      double time = (static_cast<double>(cv::getTickCount()) - start) / cv::getTickFrequency();
      RCLCPP_DEBUG(this->get_logger(), "Cloud handler cost: %f ms", time * 1000);
    } else {
      RCLCPP_INFO(this->get_logger(), "Status OBJECT_EDGE.");
      PubStatus(TrackingStatusT::OBJECT_EDGE);
    }

    // Get person position and publish
    float distance = GetDistance(ai_depth, tracked.bbox);
    std::cout << "###caldis: " << distance << std::endl;
    GeometryPoseStampedT pose;
    pose.header = pose_pub.header;
    pose.pose.position = Dis2Position(distance, cam_param, body_center);
    pose.pose.orientation.w = 1.0;

    if (0.0 != distance) {
      uncorrect_count_ = 0;
      RCLCPP_INFO(this->get_logger(), "Get pose success, correct kf. ");
      if (!filter_ptr_->initialized_) {
        filter_ptr_->Init(Convert2CV(pose.pose.position));
      } else {
        cv::Point3f posePost = filter_ptr_->Correct(Convert2CV(pose.pose.position));
        pose_pub.pose = Convert2ROS(posePost);
      }
    } else {
      uncorrect_count_++;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Cannot find depth image, cannot calculate pose. ");
    unfound_count_++;
    std::cout << "###caldis: null " << std::endl;
  }

  if (filter_ptr_->initialized_ && unfound_count_ < 10) {
    pose_pub_->publish(pose_pub);
    last_stamp_ = pose_pub.header.stamp;
    RCLCPP_INFO(
      this->get_logger(), "===Pose pub: %.5f, %.5f, %.5f",
      pose_pub.pose.position.x,
      pose_pub.pose.position.y,
      pose_pub.pose.position.z);

    double dis = sqrt(pow(pose_pub.pose.position.x, 2) + pow(pose_pub.pose.position.y, 2));
    RCLCPP_INFO(this->get_logger(), "Straight line distance from person to dog: %f", dis);
    if (dis > 3.0) {
      RCLCPP_INFO(this->get_logger(), "Status OBJECT_FAR.");
      PubStatus(TrackingStatusT::OBJECT_FAR);
    } else if (dis < 0.8) {
      RCLCPP_INFO(this->get_logger(), "Status OBJECT_NEAR.");
      PubStatus(TrackingStatusT::OBJECT_NEAR);
    }
    std::cout << "###pubpose: " << pose_pub.pose.position.x << std::endl;
  } else {
    std::cout << "###pubpose: null " << std::endl;
  }
}

float ObjectTracking::GetDistance(const cv::Mat & image, const cv::Rect2d & body_tracked)
{
  float distance = 0.0;   // unit m

  if (!image.empty()) {
    std::map<int, int> map_depth;
    for (size_t i = body_tracked.x; i < body_tracked.x + body_tracked.width; ++i) {
      for (size_t j = body_tracked.y; j < body_tracked.y + body_tracked.height; ++j) {
        if (0.0 != image.at<double>(j, i)) {
          int val = floor(image.at<double>(j, i) * 10.0);
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
    RCLCPP_DEBUG(this->get_logger(), "===sum_count: %d", sum_count);

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
      RCLCPP_ERROR(this->get_logger(), "Cloud point < 100 . ");
    }
  }

  return distance;
}

bool ObjectTracking::GetExtrinsicsParam(cv::Mat & rot_mat, cv::Mat & trans_mat)
{
  bool is_success = false;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  int i = 0;
  while (!is_success && i < 1) {
    RCLCPP_INFO(this->get_logger(), "Try transform from depth to color times %d. ", i);
    try {
      RCLCPP_INFO(this->get_logger(), "Lookup transform from depth to color. ");
      geometry_msgs::msg::TransformStamped trans = tf_buffer->lookupTransform(
        "camera_color_optical_frame", "camera_depth_optical_frame", tf2::TimePointZero,
        tf2::durationFromSec(1));
      RCLCPP_INFO(this->get_logger(), "Find transform from depth to color success. ");
      RCLCPP_INFO(
        this->get_logger(), "Rotation: x - %.9f, y - %.9f, z - %.9f, w - %.9f. ",
        trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
        trans.transform.rotation.w);
      RCLCPP_INFO(
        this->get_logger(), "Translation: x - %.9f, y - %.9f, z - %.9f. ",
        trans.transform.translation.x, trans.transform.translation.y,
        trans.transform.translation.z);
      tf2::Quaternion q(trans.transform.rotation.x, trans.transform.rotation.y,
        trans.transform.rotation.z, trans.transform.rotation.w);
      tf2::Matrix3x3 matrix;
      matrix.setRotation(q);

      rot_mat =
        (cv::Mat_<double>(3, 3) << matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0],
        matrix[1][1], matrix[1][2], matrix[2][0], matrix[2][1], matrix[2][2]);
      trans_mat =
        (cv::Mat_<double>(3, 1) << trans.transform.translation.x, trans.transform.translation.y,
        trans.transform.translation.z);
      is_success = true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    } catch (...) {
      RCLCPP_WARN(this->get_logger(), "Other exception in tf transform");
    }
    i++;
  }
  return is_success;
}

ObjectTracking::~ObjectTracking()
{
  if (trans_ptr_ != nullptr) {
    delete trans_ptr_;
  }

  if (filter_ptr_ != nullptr) {
    delete filter_ptr_;
  }

  if (handler_thread_->joinable()) {
    handler_thread_->join();
  }
}

}  // namespace cyberdog_tracking
