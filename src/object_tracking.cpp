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
  cv::Mat rotDepth2Color, transDepth2Color;
  if (!GetExtrinsicsParam(rotDepth2Color, transDepth2Color)) {
    RCLCPP_ERROR(this->get_logger(), "Get external parameter fail, cannot align depth to color. ");
  }

  // Create object
  RCLCPP_INFO(this->get_logger(), "Create object.");
  trans_ptr_ = new Transform(param_path_, rotDepth2Color, transDepth2Color, stereo_mode_);
  filter_ptr_ = new DistanceFilter();
}

void ObjectTracking::CreateSub()
{
  // Subscribe body detection topic to process
  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  auto body_callback = [this](const BodyInfo::SharedPtr msg) {
      ProcessBody(msg, this->get_logger());
    };
  RCLCPP_INFO(this->get_logger(), "Subscribing to body detection topic. ");
  body_sub_ = create_subscription<BodyInfo>("body", sub_qos, body_callback);

  // Subscribe depth image to process
  rclcpp::SensorDataQoS depth_qos;
  depth_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  auto depth_callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      ProcessDepth(msg, this->get_logger());
    };
  RCLCPP_INFO(this->get_logger(), "Subscribing to depth image topic. ");
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "camera/depth/image_rect_raw",
    depth_qos, depth_callback);

  // Subscribe realsense depth camera info
  rclcpp::SensorDataQoS info_qos;
  info_qos.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
  auto info_callback = [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      ProcessInfo(msg, this->get_logger());
    };
  RCLCPP_INFO(this->get_logger(), "Subscribing to depth camera info topic. ");
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera/depth/camera_info",
    info_qos, info_callback);
}

void ObjectTracking::CreatePub()
{
  // Publish person position
  rclcpp::ServicesQoS pub_qos;
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tracking_pose", pub_qos);
  status_pub_ = create_publisher<TrackingStatus>("tracking_status", pub_qos);
}

void ObjectTracking::ProcessDepth(
  const sensor_msgs::msg::Image::SharedPtr msg,
  rclcpp::Logger logger)
{
  RCLCPP_INFO(
    logger, "Received depth image, ts: %.9d.%.9d", msg->header.stamp.sec,
    msg->header.stamp.nanosec);

  cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(msg);
  StampedImage stampedImg;
  stampedImg.header = msg->header;
  stampedImg.image = depth_ptr->image.clone();
  std::unique_lock<std::mutex> lk(depth_mtx_);
  if (vec_stamped_depth_.size() > 6) {
    vec_stamped_depth_.erase(vec_stamped_depth_.begin());
  }
  vec_stamped_depth_.push_back(stampedImg);
}

void ObjectTracking::ProcessInfo(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg,
  rclcpp::Logger logger)
{
  RCLCPP_INFO(
    logger, "Received depth camera info msg, ts: %.9d.%.9d", msg->header.stamp.sec,
    msg->header.stamp.nanosec);
  camera_info_ = *msg;
  info_sub_ = nullptr;
}

int findNearest(const std::vector<StampedImage> & vecImg, const std_msgs::msg::Header & header)
{
  std::cout << "tofind stamp: " << header.stamp.sec << ", " << header.stamp.nanosec << std::endl;
  std::cout << "depth img stamp: " << std::endl;
  for (auto & img : vecImg) {
    std::cout << img.header.stamp.sec << ", " << img.header.stamp.nanosec << std::endl;
  }
  int position = -1;
  float minInterval = 0.1;
  for (size_t i = 0; i < vecImg.size(); ++i) {
    float interval = abs(
      static_cast<double>(vecImg[i].header.stamp.sec) - static_cast<double>(header.stamp.sec) +
      static_cast<double>(static_cast<int>(vecImg[i].header.stamp.nanosec) -
      static_cast<int>(header.stamp.nanosec)) * static_cast<double>(1.0e-9));
    if (interval < minInterval) {
      minInterval = interval;
      position = i;
    }
    std::cout << "interval: " << interval << std::endl;
  }
  return position;
}

cv::Rect toCV(const sensor_msgs::msg::RegionOfInterest & roi)
{
  cv::Rect rect = cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height);
  return rect;
}

sensor_msgs::msg::RegionOfInterest toROS(const cv::Rect & rect)
{
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = rect.x;
  roi.y_offset = rect.y;
  roi.width = rect.width;
  roi.height = rect.height;
  return roi;
}

void ObjectTracking::ProcessBody(const BodyInfo::SharedPtr msg, rclcpp::Logger logger)
{
  RCLCPP_DEBUG(
    logger, "Received detection result, ts %.9d.%.9d", msg->header.stamp.sec,
    msg->header.stamp.nanosec);
  RCLCPP_DEBUG(logger, "Received body num %d", msg->count);

  StampedBbox tracked;
  std::vector<PersonInfo> bboxDet;
  for (size_t i = 0; i < msg->count; ++i) {
    cv::Rect bbox = toCV(msg->infos[i].roi);
    PersonInfo info(bbox, msg->infos[i].reid);
    bboxDet.push_back(info);
    if (!msg->infos[i].reid.empty()) {
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
    PersonInfo personTracked;
    for (size_t i = 0; i < bodies.vecInfo.size(); ++i) {
      if (!bodies.vecInfo[i].id.empty()) {
        personTracked = bodies.vecInfo[i];
      }
    }

    // Get pose of tracked bbox and pub
    PubPose(bodies.header, personTracked);
  }
}

geometry_msgs::msg::Pose toROS(const cv::Point3f & from)
{
  geometry_msgs::msg::Pose to;
  to.position.x = from.x;
  to.position.y = from.y;
  to.position.z = from.z;
  return to;
}

geometry_msgs::msg::Point toPosition(
  float & distance, const cv::Mat & cameraParam,
  const cv::Vec2d & center)
{
  geometry_msgs::msg::Point position;
  position.x = distance;
  position.y = -distance *
    (center[0] - cameraParam.at<double>(0, 2)) / cameraParam.at<double>(0, 0);
  position.z = -distance *
    (center[1] - cameraParam.at<double>(1, 2)) / cameraParam.at<double>(1, 1);
  return position;
}

void ObjectTracking::PubStatus(const uint8_t & status)
{
  TrackingStatus trackingStatus;
  trackingStatus.status = status;
  status_pub_->publish(trackingStatus);
}

void ObjectTracking::PubPose(const std_msgs::msg::Header & header, const PersonInfo & tracked)
{
  if (tracked.id.empty()) {
    return;
  }

  // Find depth image and get distance
  cv::Mat depth_image;
  {
    std::unique_lock<std::mutex> lk(depth_mtx_);
    RCLCPP_DEBUG(
      this->get_logger(), "===To find depth image according to bbox ts: %.9d.%.9d",
      header.stamp.sec, header.stamp.nanosec);
    int position = findNearest(vec_stamped_depth_, header);
    if (position >= 0) {
      depth_image = vec_stamped_depth_[position].image.clone();
      RCLCPP_INFO(
        this->get_logger(), "===Find depth image ts:  %.9d.%.9d",
        vec_stamped_depth_[position].header.stamp.sec,
        vec_stamped_depth_[position].header.stamp.nanosec);
    }
  }

  // Get body position and publish
  geometry_msgs::msg::PoseStamped posePub;
  posePub.header.stamp = header.stamp;
  posePub.header.frame_id = "camera_link";
  cv::Mat cameraParam = trans_ptr_->ai_camera_param_;
  cv::Vec2d bodyCenter = cv::Vec2d(
    tracked.bbox.x + tracked.bbox.width / 2.0,
    tracked.bbox.y + tracked.bbox.height / 2.0);
  cv::Mat ai_depth;
  if (!depth_image.empty()) {
    unfound_count_ = 0;
    if (tracked.bbox.x > 50 && tracked.bbox.x + tracked.bbox.width < 1230) {
      RCLCPP_INFO(this->get_logger(), "Get distance according to cloud point. ");
      double start = static_cast<double>(cv::getTickCount());
      ai_depth = trans_ptr_->DepthToAi(depth_image, camera_info_, row_scale_, col_scale_);
      // imshow
      // cv::imshow("depth", ai_depth);
      // cv::waitKey(10);

      double time = (static_cast<double>(cv::getTickCount()) - start) / cv::getTickFrequency();
      RCLCPP_DEBUG(this->get_logger(), "Cloud handler cost: %f ms", time * 1000);
    } else {
      RCLCPP_INFO(this->get_logger(), "Status OBJECT_EDGE.");
      PubStatus(protocol::msg::TrackingStatus::OBJECT_EDGE);
    }

    // Get person pose and publish
    float distance = GetDistance(ai_depth, tracked.bbox);
    std::cout << "dis: " << distance << std::endl;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = posePub.header;
    pose.pose.position = toPosition(distance, cameraParam, bodyCenter);
    pose.pose.orientation.w = 1.0;

    if (0.0 != distance) {
      uncorrect_count_ = 0;
      RCLCPP_INFO(this->get_logger(), "Get pose success, correct kf. ");
      if (!filter_ptr_->initialized_ || uncorrect_count_ > 3) {
        filter_ptr_->Init(
          cv::Point3f(
            pose.pose.position.x, pose.pose.position.y,
            pose.pose.position.z));
      } else {
        filter_ptr_->Correct(
          cv::Point3f(
            pose.pose.position.x, pose.pose.position.y,
            pose.pose.position.z));
      }
    } else {
      uncorrect_count_++;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Cannot find depth image, cannot calculate pose. ");
    unfound_count_++;
  }

  if (filter_ptr_->initialized_ && uncorrect_count_ <= 3 && unfound_count_ < 10) {
    float delta = abs(
      static_cast<double>(posePub.header.stamp.sec) - static_cast<double>(last_stamp_.sec) +
      static_cast<double>(static_cast<int>(posePub.header.stamp.nanosec) -
      static_cast<int>(last_stamp_.nanosec)) * static_cast<double>(1.0e-9));
    cv::Point3f posePredict = filter_ptr_->Predict(delta);
    posePub.pose = toROS(posePredict);
    float distance = posePub.pose.position.x;
    posePub.pose.position = toPosition(distance, cameraParam, bodyCenter);
    // Add angle of current pose and 300ms after
    cv::Point3f poseFuture = filter_ptr_->Predict(0.3);
    float angle = std::atan((poseFuture.y - posePredict.y) / (poseFuture.x - posePredict.x));
    RCLCPP_INFO(this->get_logger(), "Angle with x axis: %f", angle * 180 / CV_PI);
    posePub.pose.orientation.w = angle;
    pose_pub_->publish(posePub);
    last_stamp_ = posePub.header.stamp;
    RCLCPP_INFO(
      this->get_logger(), "===Pose pub: %.5f, %.5f, %.5f",
      posePub.pose.position.x,
      posePub.pose.position.y,
      posePub.pose.position.z);

    double dis = sqrt(pow(posePub.pose.position.x, 2) + pow(posePub.pose.position.y, 2));
    RCLCPP_INFO(this->get_logger(), "Straight line distance from person to dog: %f", dis);
    if (dis > 3.0) {
      RCLCPP_INFO(this->get_logger(), "Status OBJECT_FAR.");
      PubStatus(protocol::msg::TrackingStatus::OBJECT_FAR);
    } else if (dis < 0.8) {
      RCLCPP_INFO(this->get_logger(), "Status OBJECT_NEAR.");
      PubStatus(protocol::msg::TrackingStatus::OBJECT_NEAR);
    }
  }
}

float ObjectTracking::GetDistance(const cv::Mat & image, const cv::Rect2d & body_tracked)
{
  float distance = 0.0;   // unit m

  if (!image.empty()) {
    std::map<int, int> mapDepth;
    for (size_t i = body_tracked.x; i < body_tracked.x + body_tracked.width; ++i) {
      for (size_t j = body_tracked.y; j < body_tracked.y + body_tracked.height; ++j) {
        if (0.0 != image.at<double>(j, i)) {
          int val = floor(image.at<double>(j, i) * 10.0);
          std::map<int, int>::iterator iter = mapDepth.find(val);
          if (iter != mapDepth.end()) {
            iter->second++;
          } else {
            mapDepth.insert(std::pair<int, int>(val, 1));
          }
        }
      }
    }

    int sumCount = 0;
    for (std::map<int, int>::iterator it = mapDepth.begin(); it != mapDepth.end(); ++it) {
      // std::cout << it->first << "-" << it->second << "; ";
      sumCount += it->second;
    }
    // std::cout << std::endl;
    RCLCPP_DEBUG(this->get_logger(), "===Sumcount: %d", sumCount);

    int disSum = 0;
    int disCount = 0;
    bool bFind = false;

    int lastValue = 0;
    if (!mapDepth.empty()) {
      lastValue = mapDepth.begin()->first;
    }
    if (sumCount > 100) {
      for (std::map<int, int>::iterator it = mapDepth.begin(); it != mapDepth.end() && !bFind;
        ++it)
      {
        if ((static_cast<double>(it->second) / sumCount) > 0.02 && (it->first - lastValue) < 2) {
          disCount += it->second;
          disSum += it->first / 10.0 * it->second;
        } else {
          if (disCount / static_cast<double>(sumCount) > 0.2) {
            bFind = true;
            distance = disSum / static_cast<double>(disCount);
          }
          disSum = 0;
          disCount = 0;
        }
        lastValue = it->first;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Cloud point < 100 . ");
    }
  }

  return distance;
}

bool ObjectTracking::GetExtrinsicsParam(cv::Mat & rot_mat, cv::Mat & trans_mat)
{
  bool bSuccess = false;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  int i = 0;
  while (!bSuccess && i < 3) {
    RCLCPP_INFO(this->get_logger(), "Try transform from depth to color times %d. ", i);
    try {
      RCLCPP_INFO(this->get_logger(), "Lookup transform from depth to color. ");
      geometry_msgs::msg::TransformStamped trans = tf_buffer->lookupTransform(
        "camera_color_optical_frame", "camera_depth_optical_frame", tf2::TimePointZero,
        tf2::durationFromSec(10));
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
      bSuccess = true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    } catch (...) {
      RCLCPP_WARN(this->get_logger(), "Other exception in tf transform");
    }
    i++;
  }
  return bSuccess;
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
