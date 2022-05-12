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


namespace apollo {
namespace tools {


FeatureAssociation::FeatureAssociation() {}
FeatureAssociation::~FeatureAssociation() {}

bool FeatureAssociation::Init() {
  sub_imu = node_->CreateReader<apollo::localization::CorrectedImu>(
      FLAGS_outlier_cloud_topic,
      [&](const std::shared_ptr<apollo::localization::CorrectedImu>& imu_msg) {
        OutlierCloudHandler(imu_msg);
      });

  pub_corner_points_sharp = node_->CreateWriter<apollo::drivers::PointCloud>("laser_cloud_sharp");
  pub_corner_points_less_sharp = node_->CreateWriter<apollo::drivers::PointCloud>("laser_cloud_less_sharp");
  pub_surf_points_flat = node_->CreateWriter<apollo::drivers::PointCloud>("laser_cloud_flat");
  pub_surf_points_less_flat = node_->CreateWriter<apollo::drivers::PointCloud>("laser_cloud_less_flat");
  pub_laser_cloud_corner_last = node_->CreateWriter<apollo::drivers::PointCloud>("laser_cloud_corner_last");
  pub_laser_cloud_surf_last = node_->CreateWriter<apollo::drivers::PointCloud>("laser_cloud_surf_last");
  pub_outlier_cloud_last = node_->CreateWriter<apollo::drivers::PointCloud>("outlier_cloud_last");
  pub_laser_odometry = node_->CreateWriter<nav_msgs::Odometry>("laser_odom_to_init");

  InitializationValue();

  return true;
}


void FeatureAssociation::ImuHandler(
    const std::shared_ptr<apollo::localization::CorrectedImu>& imu_in) {

}


bool FeatureAssociation::Proc() {

  return true;
}


void FeatureAssociation::AdjustDistortion() {

}

void FeatureAssociation::CalculateSmoothness() {
  int cloud_size = segmented_cloud->points.size();
  for (int i = 5; i < cloud_size - 5; ++i) {
    auto cur = seg_info.segmentedCloudRange.begin() + i;

    float diff_range = std::reduce(cur - 5, cur + 5, -11*(*cur));

    cloud_curvature[i] = diff_range * diff_range;
    cloud_neighbor_picked[i] = 0;
    cloud_label[i] = 0;

    cloud_smoothness[i].value = cloud_curvature[i];
    cloud_smoothness[i].ind = i;
  }
}


void FeatureAssociation::MarkOccludedPoints() {
  int cloud_size = segmented_cloud->points.size();
  // todo(zero): why 6 and not 5
  for (int i = 5; i < cloud_size - 6; ++i) {
    float depth1 = seg_info.segmented_cloud_range[i];
    float depth2 = seg_info.segmented_cloud_range[i+1];
    int column_diff = std::abs(static_cast<int>(
                        seg_info.segmented_cloud_col_ind[i+1] -
                        seg_info.segmented_cloud_col_ind[i]));
    if (column_diff < 10) {
      if (depth1 - depth2 > 0.3) {

      } else if (depth2 - depth1 > 0.3) {

      }
    }

    float diff1 = std::fabs(seg_info.segmented_cloud_range[i-1] - seg_info.segmented_cloud_range[i]);
    float diff2 = std::fabs(seg_info.segmented_cloud_range[i+1] - seg_info.segmented_cloud_range[i]);

    int threshold = 0.02 * seg_info.segmented_cloud_range[i];
    if (diff1 > threshold && diff2 > threshold) {
      cloud_neighbor_picked[i] = 1;
    }
  }
}


}  // namespace tools
}  // namespace apollo
