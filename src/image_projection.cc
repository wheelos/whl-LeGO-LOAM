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


#include "modules\tools\ilego_loam\include\image_projection.h"

namespace apollo {
namespace tools {

constexpr int8_t ImageProjection::dirs[][];

ImageProjection::ImageProjection() :
    nan_point(quiet_NaN(), quiet_NaN(), quiet_NaN(), -1),
    all_pushed_indX(N_SCAN * HORIZON_SCAN),
    all_pushed_indY(N_SCAN * HORIZON_SCAN),
    queue_indX(N_SCAN * HORIZON_SCAN),
    queue_indY(N_SCAN * HORIZON_SCAN) {

}

ImageProjection::~ImageProjection() {

}

bool ImageProjection::Init() {
  sub_laser_cloud = node_->CreateReader<sensor_msgs::PointCloud2>(
    pointCloudTopic,
    &ImageProjection::CloudHandler
  );

  pub_full_cloud = node_->CreateWriter<sensor_msgs::PointCloud2>("/full_cloud_projected");
  pub_full_info_cloud = node_->CreateWriter<sensor_msgs::PointCloud2>("/full_cloud_info");

  pub_ground_cloud = node_->CreateWriter<sensor_msgs::PointCloud2>("/ground_cloud");
  pub_segmented_cloud = node_->CreateWriter<sensor_msgs::PointCloud2>("/segmented_cloud");
  pub_segmented_cloud_pure = node_->CreateWriter<sensor_msgs::PointCloud2>("/segmented_cloud_pure");
  pub_segmented_cloud_info = node_->CreateWriter<cloud_msgs::CloudInfo>("/segmented_cloud_info");
  pub_outlier_cloud = node_->CreateWriter<sensor_msgs::PointCloud2>("/outlier_cloud");
}

void ImageProjection::AllocateMemory() {
  laser_cloud_in = std::make_shared<pcl::PointCloud<PointType>>();
  laser_cloud_in_ring = std::make_shared<pcl::PointCloud<PointXYZIR>>();

  full_cloud = std::make_shared<pcl::PointCloud<PointType>>();
  full_info_cloud = std::make_shared<pcl::PointCloud<PointType>>();

  ground_cloud = std::make_shared<pcl::PointCloud<PointType>>();
  segmented_cloud = std::make_shared<pcl::PointCloud<PointType>>();
  segmented_cloud_pure = std::make_shared<pcl::PointCloud<PointType>>();
  outlier_cloud = std::make_shared<pcl::PointCloud<PointType>>();

  full_cloud->points.resize(N_SCAN * HORIZON_SCAN);
  full_info_cloud->points.resize(N_SCAN * HORIZON_SCAN);

  seg_msg = std::make_shared<cloud_msgs::CloudInfo>();
  seg_msg->startRingIndex = N_SCAN;
  seg_msg->endRingIndex = N_SCAN;

  seg_msg->segmentedCloudGroundFlag = N_SCAN * HORIZON_SCAN;
  seg_msg->segmentedCloudColInd = N_SCAN * HORIZON_SCAN;
  seg_msg->segmentedCloudRange = N_SCAN * HORIZON_SCAN;
}

void ImageProjection::ResetParameters() {
  laser_cloud_in->clear();
}

void ImageProjection::CloudHandler(ConstPointCloud2Ptr& laser_cloud_msg) {

}


}  // namespace tools
}  // namespace apollo
