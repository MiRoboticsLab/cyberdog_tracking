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

#include "cyberdog_tracking/transform.hpp"

#include <string>
#include <vector>

namespace cyberdog_tracking
{

Transform::Transform(
  const std::string & file_path, const cv::Mat & rot, const cv::Mat & trans,
  bool stereo_mode)
{
  try {
    if (rot.empty() || trans.empty()) {
      std::cout << "Get extrinsics param fail, using default param. " << std::endl;
      depth2color_r_ =
        (cv::Mat_<double>(3, 3) << 0.9999962449073792, 0.0003213161835446954,
        -0.0027217825409024954, -0.0003334041975904256, 0.9999901056289673, -0.0044419169425964355,
        0.0027203282807022333, 0.004442807752639055, 0.9999864101409912);
      depth2color_t_ =
        (cv::Mat_<double>(3, 1) << -0.059059660881757736, 4.421528137754649e-05,
        6.807535100961104e-05);
    } else {
      depth2color_r_ = rot;
      depth2color_t_ = trans;
    }
  } catch (...) {
    depth2color_r_ =
      (cv::Mat_<double>(3, 3) << 0.9999962449073792, 0.0003213161835446954,
      -0.0027217825409024954, -0.0003334041975904256, 0.9999901056289673, -0.0044419169425964355,
      0.0027203282807022333, 0.004442807752639055, 0.9999864101409912);
    depth2color_t_ =
      (cv::Mat_<double>(3, 1) << -0.059059660881757736, 4.421528137754649e-05,
      6.807535100961104e-05);
  }

  stereo_mode_ = stereo_mode;
  LoadParameters(file_path);
}

cv::Mat Transform::DepthToAi(
  const cv::Mat & depth_image,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const float & rows_scale, const float & cols_scale)
{
  cv::Mat ai_image;
  if (stereo_mode_) {
    ai_image = StereoDepthToAi(depth_image, camera_info, rows_scale, cols_scale);
  } else {
    ai_image = RealsenseDepthToAi(depth_image, camera_info, rows_scale, cols_scale);
  }
  return ai_image;
}

cv::Mat Transform::RealsenseDepthToAi(
  const cv::Mat & depth_image,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const float & rows_scale, const float & cols_scale)
{
  fx_ = camera_info.p[0];
  fy_ = camera_info.p[5];
  cx_ = camera_info.p[2];
  cy_ = camera_info.p[6];

  cv::Mat ai_depth(ai_img_size_, CV_64FC1, cv::Scalar(0));
  int depth_rows = depth_image.rows * rows_scale;
  int depth_cols = depth_image.cols * cols_scale;
  int rows = depth_rows * 0.5;

  std::future<void> ft1 = std::async(
    std::launch::async, [&] {
      for (int r = 0; r < rows; r += 3) {
        for (int c = 0; c < depth_cols; c += 3) {
          double depth = depth_image.at<ushort>(r, c) / 1000.;
          if (depth <= 0) {continue;}
          DepthToAi(ai_depth, depth, r, c);
        }
      }
    });

  std::future<void> ft2 = std::async(
    std::launch::async, [&] {
      for (int r = rows; r < depth_rows; r += 3) {
        for (int c = 0; c < depth_cols; c += 3) {
          double depth = depth_image.at<ushort>(r, c) / 1000.;
          if (depth <= 0) {continue;}
          DepthToAi(ai_depth, depth, r, c);
        }
      }
    });

  ft1.wait();
  ft2.wait();

  return ai_depth;
}

cv::Mat Transform::StereoDepthToAi(
  const cv::Mat & depth_image, const sensor_msgs::msg::CameraInfo,
  const float & rows_scale, const float & cols_scale)
{
  fx_ = 220;
  fy_ = 220;
  cx_ = 319.5;
  cy_ = 239.5;

  cv::Mat ai_depth(ai_img_size_, CV_64FC1, cv::Scalar(0));
  int depth_rows = depth_image.rows * rows_scale;
  int depth_cols = depth_image.cols * cols_scale;
  int rows = depth_rows * 0.5;

  std::future<void> ft1 = std::async(
    std::launch::async, [&] {
      for (int r = 0; r < rows; r += 1) {
        for (int c = 0; c < depth_cols; c += 1) {
          double depth = depth_image.at<float>(r, c);
          if (depth <= 0) {continue;}
          StereoDepthToAi(ai_depth, depth, r, c);
        }
      }
    });

  std::future<void> ft2 = std::async(
    std::launch::async, [&] {
      for (int r = rows; r < depth_rows; r += 1) {
        for (int c = 0; c < depth_cols; c += 1) {
          double depth = depth_image.at<float>(r, c);
          if (depth <= 0) {continue;}
          StereoDepthToAi(ai_depth, depth, r, c);
        }
      }
    });

  ft1.wait();
  ft2.wait();

  return ai_depth;
}

void Transform::DepthToAi(cv::Mat & ai_depth, double depth, int r, int c)
{
  double e = (c - cx_) / fx_;
  double f = (r - cy_) / fy_;

  double P_x0 = e * depth;
  double P_y0 = f * depth;
  double P_z0 = depth;

  double P_x1 = P_x0 * depth2color_r_.at<double>(0, 0) +
    P_y0 * depth2color_r_.at<double>(0, 1) +
    P_z0 * depth2color_r_.at<double>(0, 2) +
    depth2color_t_.at<double>(0, 0);

  double P_y1 = P_x0 * depth2color_r_.at<double>(1, 0) +
    P_y0 * depth2color_r_.at<double>(1, 1) +
    P_z0 * depth2color_r_.at<double>(1, 2) +
    depth2color_t_.at<double>(1, 0);

  double P_z1 = P_x0 * depth2color_r_.at<double>(2, 0) +
    P_y0 * depth2color_r_.at<double>(2, 1) +
    P_z0 * depth2color_r_.at<double>(2, 2) +
    depth2color_t_.at<double>(2, 0);

  double P_x2 = P_x1 * aiR_.at<double>(0, 0) +
    P_y1 * aiR_.at<double>(0, 1) +
    P_z1 * aiR_.at<double>(0, 2) +
    aiT_.at<double>(0, 0);

  double P_y2 = P_x1 * aiR_.at<double>(1, 0) +
    P_y1 * aiR_.at<double>(1, 1) +
    P_z1 * aiR_.at<double>(1, 2) +
    aiT_.at<double>(1, 0);

  double P_z2 = P_x1 * aiR_.at<double>(2, 0) +
    P_y1 * aiR_.at<double>(2, 1) +
    P_z1 * aiR_.at<double>(2, 2) +
    aiT_.at<double>(2, 0);

  double P_norm = std::sqrt(P_x2 * P_x2 + P_y2 * P_y2 + P_z2 * P_z2);
  double z = P_z2 + aiXi_.at<double>(0, 0) * P_norm;
  double p_x = P_x2 / z;
  double p_y = P_y2 / z;

  // Apply generalised projection matrix
  double p_u = aiK_.at<double>(0, 0) * p_x + aiK_.at<double>(0, 2);
  double p_v = aiK_.at<double>(1, 1) * p_y + aiK_.at<double>(1, 2);

  cv::Point point = cv::Point2d(static_cast<int>(p_u), static_cast<int>(p_v));
  if (point.x > 0 && point.x < ai_depth.cols && point.y > 0 && point.y < ai_depth.rows) {
    ai_depth.at<double>(point) = P_z1;
  }
}

void Transform::StereoDepthToAi(cv::Mat & ai_depth, double depth, int r, int c)
{
  double e = (c - cx_) / fx_;
  double f = (r - cy_) / fy_;

  double P_x0 = e * depth;
  double P_y0 = f * depth;
  double P_z0 = depth;

  double P_x1 = P_x0 * laiR_.at<double>(0, 0) +
    P_y0 * laiR_.at<double>(0, 1) +
    P_z0 * laiR_.at<double>(0, 2) +
    laiT_.at<double>(0, 0);

  double P_y1 = P_x0 * laiR_.at<double>(1, 0) +
    P_y0 * laiR_.at<double>(1, 1) +
    P_z0 * laiR_.at<double>(1, 2) +
    laiT_.at<double>(1, 0);

  double P_z1 = P_x0 * laiR_.at<double>(2, 0) +
    P_y0 * laiR_.at<double>(2, 1) +
    P_z0 * laiR_.at<double>(2, 2) +
    laiT_.at<double>(2, 0);

  double P_norm = std::sqrt(P_x1 * P_x1 + P_y1 * P_y1 + P_z1 * P_z1);
  double z = P_z1 + aiXi_.at<double>(0, 0) * P_norm;
  double p_x = P_x1 / z;
  double p_y = P_y1 / z;

  // Apply generalised projection matrix
  double p_u = aiK_.at<double>(0, 0) * p_x + aiK_.at<double>(0, 2);
  double p_v = aiK_.at<double>(1, 1) * p_y + aiK_.at<double>(1, 2);

  cv::Point point = cv::Point2d(static_cast<int>(p_u), static_cast<int>(p_v));
  if (point.x > 0 && point.x < ai_depth.cols && point.y > 0 && point.y < ai_depth.rows) {
    ai_depth.at<double>(point) = P_z1;
  }
}

void Transform::LoadParameters(const std::string & file_path)
{
  std::string AI_file = file_path + "/camera_AI.yaml";
  std::string AI_extrinsic = file_path + "/extrinsics_ColorAI.yaml";
  std::string LeftAI_extrinsic = file_path + "/extrinsics_LeftAI.yaml";

  // AI intrinsic extrinsic(AI->left)
  if ((!AI_file.empty()) && (!AI_extrinsic.empty()) && (!LeftAI_extrinsic.empty())) {
    cv::Size img_size;
    LoadIntrinsic(AI_file, aiK_, aiD_, aiXi_, img_size);
    ai_img_size_ = img_size;

    cv::Mat rc, tc;
    LoadExtrinsic(AI_extrinsic, rc, tc);
    rc.copyTo(aiR_);
    tc.copyTo(aiT_);

    cv::Mat rl, tl;
    LoadExtrinsic(LeftAI_extrinsic, rl, tl);
    rl.copyTo(laiR_);
    tl.copyTo(laiT_);
  } else {
    std::cout << "AI params is null , can not use AI function!" << std::endl;
  }
}

void Transform::LoadIntrinsic(
  const std::string & file_name, cv::Mat & K, cv::Mat & D, cv::Mat & Xi,
  cv::Size & img_size)
{
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "The file " + file_name << " is not exist!" << std::endl;
    exit(0);
  } else {
    std::cout << "Open " + file_name + " file success!" << std::endl;
  }

  int width = static_cast<int>(fs["image_width"]);
  int height = static_cast<int>(fs["image_height"]);
  cv::FileNode n = fs["mirror_parameters"];
  double xi = static_cast<double>(n["xi"]);
  cv::FileNode n1 = fs["distortion_parameters"];
  double k1 = static_cast<double>(n1["k1"]);
  double k2 = static_cast<double>(n1["k2"]);
  double p1 = static_cast<double>(n1["p1"]);
  double p2 = static_cast<double>(n1["p2"]);
  cv::FileNode n2 = fs["projection_parameters"];
  double gamma1 = static_cast<double>(n2["gamma1"]);
  double gamma2 = static_cast<double>(n2["gamma2"]);
  double u0 = static_cast<double>(n2["u0"]);
  double v0 = static_cast<double>(n2["v0"]);
  cv::Mat kMat = cv::Mat::zeros(3, 3, CV_64F);
  kMat.at<double>(0, 0) = gamma1;
  kMat.at<double>(0, 2) = u0;
  kMat.at<double>(1, 1) = gamma2;
  kMat.at<double>(1, 2) = v0;
  kMat.at<double>(2, 2) = 1;
  cv::Mat dMat = (cv::Mat_<double>(4, 1) << k1, k2, p1, p2);
  cv::Mat xiMat = (cv::Mat_<double>(1, 1) << xi);
  img_size = cv::Size(width, height);
  kMat.copyTo(K);
  dMat.copyTo(D);
  xiMat.copyTo(Xi);
  kMat.copyTo(ai_camera_param_);
}

void Transform::LoadExtrinsic(const std::string & file_name, cv::Mat & rot, cv::Mat & trans)
{
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "The file " + file_name + " is not exist!" << std::endl;
    exit(0);
  } else {
    std::cout << "Open " + file_name + " file success. " << std::endl;
  }

  double t_x = fs["transform"]["t_x"];
  double t_y = fs["transform"]["t_y"];
  double t_z = fs["transform"]["t_z"];

  std::vector<double> t_r;
  fs["transform"]["r"] >> t_r;
  fs.release();
  cv::Mat rMat = cv::Mat::zeros(3, 3, CV_64F);
  for (int r = 0; r < rMat.rows; ++r) {
    for (int c = 0; c < rMat.cols; ++c) {
      rMat.at<double>(r, c) = t_r[r * 3 + c];
    }
  }
  cv::Mat tMat = (cv::Mat_<double>(3, 1) << t_x, t_y, t_z);
  rMat.copyTo(rot);
  tMat.copyTo(trans);
}

Transform::~Transform() {}

}  // namespace cyberdog_tracking
