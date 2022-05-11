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


#include "pcl/point_types.h"


namespace apollo {
namespace tools {


extern static const int N_SCAN = 16;
extern static const int HORIZON_SCAN = 1800;

extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0 + 0.1;
extern const int groundScanInd = 7;



using PointCloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
using PointCloudRPtr = pcl::PointCloud<pcl::PointXYZIR>::Ptr;
using DriverPointCloudPtr = std::shared_ptr<drivers::PointCloud>;


void ToPclPointCloud(const DriverPointCloudPtr& from, const PointCloudPtr& to) {
  for (int i = 0; i < from->point.size(); ++i) {
    pcl::PointXYZI point(from->point.x,
                         from->point.y,
                         from->point.z,
                         from->point.intensity);
    to->push_back(point);
  }
}

void ToPclPointCloud(const DriverPointCloudPtr& from, const PointCloudRPtr& to) {
  for (int i = 0; i < from->point.size(); ++i) {
    pcl::PointXYZIR point(from->point.x,
                          from->point.y,
                          from->point.z,
                          from->point.intensity);
    to->push_back(point);
  }
  to->is_dense = laser_cloud_msg->is_dense;
}


void ToDriverPointCloud(const PointCloudPtr& from, const drivers::PointCloud& to) {
  for (int i = 0; i < from->point.size(); ++i) {
    apollo::drivers::PointXYZI point(from->point.x,
                                     from->point.y,
                                     from->point.z,
                                     from->point.intensity);
    to.point.Add(point);
  }
}


bool IsNaN(const pcl::PointXYZI& point) {
  return (!std::isfinite(point.x) ||
          !std::isfinite(point.y) ||
          !std::isfinite(point.z));
}

}  // namespace tools
}  // namespace apollo
