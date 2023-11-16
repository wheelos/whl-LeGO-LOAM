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


#pragma once


#include <numeric>

#include "cyber/cyber.h"
#include "modules/tools/ilego_loam/proto/cloud_info.pb.h"

#include "modules/tools/ilego_loam/src/utility.h"


namespace apollo {
namespace tools {


class FeatureAssociation final : public apollo::cyber::TimerComponent {
 public:
  using PointCloudReader = std::shared_ptr<cyber::Reader<apollo::drivers::PointCloud>>;
  using PointCloudWriter = std::shared_ptr<cyber::Writer<apollo::drivers::PointCloud>>;
  using CorrectedImuReader = std::shared_ptr<cyber::Reader<apollo::localization::CorrectedImu>>;
  using OdometryWriter = std::shared_ptr<cyber::Writer<nav_msgs::Odometry>>;

  FeatureAssociation();
  ~FeatureAssociation();

  bool Init() override;
  bool Proc() override;
 private:


 private:
  // todo(zero): check if same as sensor_msgs::Imu?
  PointCloudReader sub_segmented_cloud;
  std::shared_ptr<cyber::Reader<cloud_msgs::CloudInfo>> sub_segmented_cloud_info;
  PointCloudReader sub_outlier_cloud;
  CorrectedImuReader sub_imu;

  PointCloudWriter pub_corner_points_sharp;
  PointCloudWriter pub_corner_points_less_sharp;
  PointCloudWriter pub_surf_points_flat;
  PointCloudWriter pub_surf_points_less_flat;
  PointCloudWriter pub_laser_cloud_corner_last;
  PointCloudWriter pub_laser_cloud_surf_last;
  PointCloudWriter pub_outlier_cloud_last;
  OdometryWriter pub_laser_odometry;

  pcl::VoxelGrid<PointType> downSizeFilter;

  double timeScanCur;
  double timeNewSegmentedCloud;
  double timeNewSegmentedCloudInfo;
  double timeNewOutlierCloud;

  bool newSegmentedCloud;
  bool newSegmentedCloudInfo;
  bool newOutlierCloud;

  cloud_msgs::CloudInfo seg_info;
  apollo::common::Header cloud_header;
};

CYBER_REGISTER_COMPONENT(FeatureAssociation)

}  // namespace tools
}  // namespace apollo
