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


#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/filter.h"

#include "modules/drivers/proto/pointcloud.pb.h"

namespace apollo {
namespace tools {


extern const int N_SCAN = 16;
extern const int HORIZON_SCAN = 1800;

extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0 + 0.1;
extern const int groundScanInd = 7;


extern const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


using PointType = pcl::PointXYZI;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
// using PointCloudRPtr = pcl::PointCloud<pcl::PointXYZIR>::Ptr;
using DriverPointCloudPtr = std::shared_ptr<apollo::drivers::PointCloud>;


struct smoothness_t {
  float value;
  size_t ind;
};

struct by_value {
  bool operator()(const smoothness_t& left, const smoothness_t& right) {
    return left.value < right.value;
  }
};


void ToPclPointCloud(const DriverPointCloudPtr& from, const PointCloudPtr& to) {
  for (int i = 0; i < from->point().size(); ++i) {
    pcl::PointXYZI point(from->point(i).x(),
                         from->point(i).y(),
                         from->point(i).z(),
                         from->point(i).intensity());
    to->push_back(point);
  }
  to->is_dense = from->is_dense();
}

void ToDriverPointCloud(const PointCloudPtr& from, apollo::drivers::PointCloud& to) {
  for (size_t i = 0; i < from->points.size(); ++i) {
    auto pb_point = to.add_point();
    pb_point->set_x(from->points[i].x);
    pb_point->set_y(from->points[i].y);
    pb_point->set_z(from->points[i].z);
    pb_point->set_intensity(from->points[i].intensity);
  }
}


bool IsNaN(const pcl::PointXYZI& point) {
  return (!std::isfinite(point.x) ||
          !std::isfinite(point.y) ||
          !std::isfinite(point.z));
}

double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

}  // namespace tools
}  // namespace apollo
