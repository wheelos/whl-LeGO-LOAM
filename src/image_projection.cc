// Copyright 2022 daohu527@gmail.com
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

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.


//  Created Date: 2022-5-5
//  Author: daohu527


#include "modules/tools/ilego_loam/flags/lego_loam_gflags.h"
#include "modules/tools/ilego_loam/src/image_projection.h"


namespace apollo {
namespace tools {

static constexpr int8_t dirs[4][2] = {{-1, 0}, {0, 1}, {0, -1}, {1, 0}};
static constexpr int LABEL_INIT = 0;
static constexpr int LABEL_INVALID = INT_MAX;

static const pcl::PointXYZI nan_point(
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    -1);


ImageProjection::ImageProjection() {}

ImageProjection::~ImageProjection() {}

bool ImageProjection::Init() {
  sub_laser_cloud = node_->CreateReader<apollo::drivers::PointCloud>(
      FLAGS_lidar_topic,
      [&](const DriverPointCloudPtr& point_cloud){
        // todo(zero): check need lock???
        // std::lock_guard<std::mutex> lock(mutex_);
        CloudHandler(point_cloud);
      });

  pub_full_cloud = node_->CreateWriter<apollo::drivers::PointCloud>("/full_cloud_projected");
  pub_full_info_cloud = node_->CreateWriter<apollo::drivers::PointCloud>("/full_cloud_info");

  pub_ground_cloud = node_->CreateWriter<apollo::drivers::PointCloud>("/ground_cloud");
  pub_segmented_cloud = node_->CreateWriter<apollo::drivers::PointCloud>("/segmented_cloud");
  pub_segmented_cloud_pure = node_->CreateWriter<apollo::drivers::PointCloud>("/segmented_cloud_pure");
  pub_segmented_cloud_info = node_->CreateWriter<cloud_msgs::CloudInfo>("/segmented_cloud_info");
  pub_outlier_cloud = node_->CreateWriter<apollo::drivers::PointCloud>("/outlier_cloud");

  AllocateMemory();
  ResetParameters();
  return true;
}

void ImageProjection::AllocateMemory() {
  laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  // todo(zero): need PointXYZIR
  laser_cloud_in_ring.reset(new pcl::PointCloud<PointXYZIR>());

  full_cloud.reset(new pcl::PointCloud<PointType>());
  full_info_cloud.reset(new pcl::PointCloud<PointType>());
  full_cloud->points.resize(N_SCAN * HORIZON_SCAN);
  full_info_cloud->points.resize(N_SCAN * HORIZON_SCAN);

  ground_cloud.reset(new pcl::PointCloud<PointType>());

  segmented_cloud.reset(new pcl::PointCloud<PointType>());
  segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  outlier_cloud.reset(new pcl::PointCloud<PointType>());

  // todo(zero): need to set default value
  seg_msg.mutable_start_ring_index()->Resize(N_SCAN, 0);
  seg_msg.mutable_end_ring_index()->Resize(N_SCAN, 0);

  seg_msg.mutable_segmented_cloud_ground_flag()->Resize(N_SCAN * HORIZON_SCAN, false);
  seg_msg.mutable_segmented_cloud_col_ind()->Resize(N_SCAN * HORIZON_SCAN, 0);
  seg_msg.mutable_segmented_cloud_range()->Resize(N_SCAN * HORIZON_SCAN, 0);

  cluster.reserve(N_SCAN * HORIZON_SCAN);
}

void ImageProjection::ResetParameters() {
  laser_cloud_in->clear();
  laser_cloud_in_ring->clear();
  ground_cloud->clear();
  segmented_cloud->clear();
  segmented_cloud_pure->clear();
  outlier_cloud->clear();

  range_mat = cv::Mat(N_SCAN, HORIZON_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
  ground_mat = cv::Mat(N_SCAN, HORIZON_SCAN, CV_8S, cv::Scalar::all(0));
  label_mat = cv::Mat(N_SCAN, HORIZON_SCAN, CV_32S, cv::Scalar::all(LABEL_INIT));
  label_count = 1;

  std::fill(full_cloud->points.begin(), full_cloud->points.end(), nan_point);
  std::fill(full_info_cloud->points.begin(), full_info_cloud->points.end(), nan_point);
}

void ImageProjection::CopyPointCloud(const DriverPointCloudPtr& laser_cloud_msg) {
  cloud_header.CopyFrom(laser_cloud_msg->header());
  ToPclPointCloud(laser_cloud_msg, laser_cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laser_cloud_in, *laser_cloud_in, indices);
  if (FLAGS_use_cloud_ring) {
    ToPclPointCloud(laser_cloud_msg, laser_cloud_in_ring);
    ACHECK(laser_cloud_in_ring->is_dense) <<
        "Point cloud is not in dense format, please remove NaN points first!";
  }
}

void ImageProjection::FindStartEndAngle() {
  // todo(zero): need check coor
  seg_msg.set_start_orientation(atan2(laser_cloud_in->points.front().y, laser_cloud_in->points.front().x));
  seg_msg.set_end_orientation(atan2(laser_cloud_in->points.back().y, laser_cloud_in->points.back().x) + 2 * M_PI);

  seg_msg.set_orientation_diff(seg_msg.end_orientation() - seg_msg.start_orientation());
  CHECK(seg_msg.orientation_diff() < 3 * M_PI) << "Point cloud orientation diff >= 3*M_PI";
  CHECK(seg_msg.orientation_diff() > M_PI) << "Point cloud orientation diff <= M_PI";
}

void ImageProjection::ProjectPointCloud() {
  for (size_t i = 0; i < laser_cloud_in->points.size(); ++i) {
    PointType this_point = laser_cloud_in->points[i];

    // Find the row and column index in the iamge for this point
    size_t row_idn;
    if (FLAGS_use_cloud_ring) {
      row_idn = laser_cloud_in_ring->points[i].ring;
    } else {
      float vertical_angle = atan2(this_point.z, sqrt(this_point.x*this_point.x + this_point.y*this_point.y)) * 180 / M_PI;
      row_idn = (vertical_angle + ang_bottom) / ang_res_y;
    }

    if (row_idn >= N_SCAN)
      continue;

    float horizon_angle = atan2(this_point.x, this_point.y) * 180 / M_PI;
    // todo(zero): horizon_angle [-180, 180], check
    size_t column_idn = round(horizon_angle / ang_res_x) + HORIZON_SCAN / 2;
    if (column_idn >= HORIZON_SCAN)
      column_idn -= HORIZON_SCAN;

    if (column_idn >= HORIZON_SCAN)
      continue;

    float range = sqrt(this_point.x*this_point.x + this_point.y*this_point.y + this_point.z*this_point.z);
    if (range < FLAGS_sensor_minimum_range)
      continue;

    range_mat.at<float>(row_idn, column_idn) = range;
    this_point.intensity = static_cast<float>(row_idn) +
        static_cast<float>(column_idn) / 10000;

    size_t index = column_idn + row_idn * HORIZON_SCAN;

    full_cloud->points[index] = this_point;
    full_info_cloud->points[index] = this_point;
    full_info_cloud->points[index].intensity = range;
  }
}

void ImageProjection::GroundRemoval() {
  // todo(zero): groundScanInd to kGroundScanInd
  for (size_t i = 0; i < groundScanInd; ++i) {
    for (size_t j = 0; j < HORIZON_SCAN; ++j) {
      size_t lower_ind = j + i * HORIZON_SCAN;
      size_t upper_ind = j + (i + 1) * HORIZON_SCAN;

      if (IsNaN(full_cloud->points[lower_ind]) ||
          IsNaN(full_cloud->points[upper_ind])) {
        ground_mat.at<int8_t>(i, j) = -1;
        continue;
      }

      float diffx = full_cloud->points[upper_ind].x - full_cloud->points[lower_ind].x;
      float diffy = full_cloud->points[upper_ind].y - full_cloud->points[lower_ind].y;
      float diffz = full_cloud->points[upper_ind].z - full_cloud->points[lower_ind].z;

      float angle = atan2(diffz, sqrt(diffx*diffx + diffy*diffy)) * 180 / M_PI;

      if (fabs(angle - FLAGS_sensor_mount_angle) <= 10) {
        ground_mat.at<int8_t>(i, j) = 1;
        ground_mat.at<int8_t>(i + 1, j) = 1;
      }
    }
  }

  for (size_t i = 0; i < N_SCAN; ++i) {
    for (size_t j = 0; j < HORIZON_SCAN; ++j) {
      if (ground_mat.at<int8_t>(i, j) == 1 || range_mat.at<float>(i, j) == FLT_MAX) {
        label_mat.at<int>(i, j) = -1;
      }
    }
  }

  for (size_t i = 0; i <= groundScanInd; ++i) {
    for (size_t j = 0; j < HORIZON_SCAN; ++j) {
      if (ground_mat.at<int8_t>(i, j) == 1)
        ground_cloud->push_back(full_cloud->points[j + i * HORIZON_SCAN]);
    }
  }
}

void ImageProjection::CloudSegmentation() {
  for (size_t i = 0; i < N_SCAN; ++i) {
    for (size_t j = 0; j < HORIZON_SCAN; ++j) {
      if (label_mat.at<int>(i, j) == LABEL_INIT)
        LabelComponents(i, j);
    }
  }

  int size_of_seg_cloud = 0;
  // Extract segmented cloud for lidar odometry
  for (size_t i = 0; i < N_SCAN; ++i) {
    seg_msg.set_start_ring_index(i, size_of_seg_cloud - 1 + 5);
    for (size_t j = 0; j < HORIZON_SCAN; ++j) {
      if (label_mat.at<int>(i, j) > 0 || ground_mat.at<int8_t>(i, j) == 1) {
        if (label_mat.at<int>(i, j) == LABEL_INVALID) {
          if (i > groundScanInd && j % 5 == 0) {
            outlier_cloud->push_back(full_cloud->points[j + i * HORIZON_SCAN]);
          }
          continue;
        }

        if (ground_mat.at<int8_t>(i, j) == 1) {
          if (j%5 != 0 && j > 5 && j < HORIZON_SCAN-5)
            continue;
        }

        seg_msg.set_segmented_cloud_ground_flag(size_of_seg_cloud, ground_mat.at<int8_t>(i, j) == 1);
        seg_msg.set_segmented_cloud_col_ind(size_of_seg_cloud, j);
        seg_msg.set_segmented_cloud_range(size_of_seg_cloud, range_mat.at<float>(i, j));
        segmented_cloud->push_back(full_cloud->points[j + i * HORIZON_SCAN]);
        ++size_of_seg_cloud;
        AINFO << "size_of_seg_cloud: " << size_of_seg_cloud;
      }
    }
    seg_msg.set_end_ring_index(i, size_of_seg_cloud - 1 - 5);
  }

  for (size_t i = 0; i < N_SCAN; ++i) {
    for (size_t j = 0; j < HORIZON_SCAN; ++j) {
      if (label_mat.at<int>(i, j) > 0 && label_mat.at<int>(i, j) != LABEL_INVALID) {
        segmented_cloud_pure->push_back(full_cloud->points[j + i*HORIZON_SCAN]);
        segmented_cloud_pure->points.back().intensity = label_mat.at<int>(i, j);
      }
    }
  }
}

void ImageProjection::PublishCloud() {
  // todo(zero): check the header
  seg_msg.mutable_header()->set_time(cloud_header.timestamp_sec());
  pub_segmented_cloud_info->Write(seg_msg);

  apollo::drivers::PointCloud laser_cloud_temp;
  ToDriverPointCloud(outlier_cloud, laser_cloud_temp);
  laser_cloud_temp.mutable_header()->set_timestamp_sec(cloud_header.timestamp_sec());
  laser_cloud_temp.mutable_header()->set_frame_id("base_link");
  pub_outlier_cloud->Write(laser_cloud_temp);

  laser_cloud_temp.mutable_point()->Clear();
  ToDriverPointCloud(segmented_cloud, laser_cloud_temp);
  pub_segmented_cloud->Write(laser_cloud_temp);

  laser_cloud_temp.mutable_point()->Clear();
  ToDriverPointCloud(full_cloud, laser_cloud_temp);
  pub_full_cloud->Write(laser_cloud_temp);

  laser_cloud_temp.mutable_point()->Clear();
  ToDriverPointCloud(full_info_cloud, laser_cloud_temp);
  pub_full_info_cloud->Write(laser_cloud_temp);

  laser_cloud_temp.mutable_point()->Clear();
  ToDriverPointCloud(ground_cloud, laser_cloud_temp);
  pub_ground_cloud->Write(laser_cloud_temp);

  laser_cloud_temp.mutable_point()->Clear();
  ToDriverPointCloud(segmented_cloud_pure, laser_cloud_temp);
  pub_segmented_cloud_pure->Write(laser_cloud_temp);
}

void ImageProjection::LabelComponents(int row, int col) {
  bool line_count_flag[N_SCAN] = {false};

  st.push({row, col});
  cluster.clear();

  while(!st.empty()) {
    auto& [c_row, c_col] = st.top();
    st.pop();

    cluster.push_back({c_row, c_col});
    label_mat.at<int>(c_row, c_col) = label_count;

    for (int i = 0; i < 4; ++i) {
      int n_row = c_row + dirs[i][0];
      int n_col = c_col + dirs[i][1];

      if (n_row >= 0 && n_row < N_SCAN && n_col >= 0 && n_col < HORIZON_SCAN &&
          label_mat.at<int>(n_row, n_col) == LABEL_INIT) {
        auto [d2, d1] = std::minmax(range_mat.at<float>(c_row, c_col),
            range_mat.at<float>(n_row, n_col));

        float alpha = dirs[i][0] == 0 ? segmentAlphaX : segmentAlphaY;
        float angle = atan2(d2 * sin(alpha), d1 - d2 * cos(alpha));

        if (angle > segmentTheta) {
          st.push({n_row, n_col});
          label_mat.at<int>(n_row, n_col) = label_count;
          line_count_flag[n_row] = true;
        }
      }
    }
  }

  bool feasible_segment = false;
  if (cluster.size() >= 30) {
    feasible_segment = true;
  } else if (cluster.size() >= segmentValidPointNum) {
    int line_count = 0;
    for (size_t i = 0; i < N_SCAN; ++i) {
      if (line_count_flag[i]) {
        ++line_count;
      }
    }

    if (line_count >= segmentValidLineNum)
      feasible_segment = true;
  }

  if (feasible_segment) {
    ++label_count;
  } else {
    for (auto& [l_row, l_col] : cluster) {
      label_mat.at<int>(l_row, l_col) = LABEL_INVALID;
    }
  }
}

void ImageProjection::CloudHandler(const DriverPointCloudPtr& laser_cloud_msg) {
  // 1. convert ros message to pcl point cloud
  CopyPointCloud(laser_cloud_msg);
  // 2. start and end angle of a scan
  FindStartEndAngle();
  // 3. range image projection
  ProjectPointCloud();
  // 4. mark ground points
  GroundRemoval();
  // 5. point cloud segmentation
  CloudSegmentation();
  // 6. publish all clouds
  PublishCloud();
  // 7. reset parameters for next iteration
  ResetParameters();
}


}  // namespace tools
}  // namespace apollo
