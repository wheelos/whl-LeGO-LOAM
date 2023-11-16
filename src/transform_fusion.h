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

#include <memory>

#include "cyber/cyber.h"

#include "src/utility.h"

namespace apollo {
namespace tools {

struct Transform {
  Eigen::Translation3d translation;
  Eigen::Quaterniond rotation;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class TransformFusion final : public cyber::Component<> {
 public:
  TransformFusion();
  ~TransformFusion();

  bool Init() override;

 private:
  void TransformAssociateToMap();
  void LaserOdometryHandler(const std::shared_ptr<nav_msgs::Odometry>& laser_odometry);
  void OdomAftMappedHandler(const std::shared_ptr<nav_msgs::Odometry>& laser_odometry);

 private:
  std::shared_ptr<cyber::Writer<nav_msgs::Odometry>> pub_laser_odometry2;
  std::shared_ptr<cyber::Reader<nav_msgs::Odometry>> sub_laser_odometry;
  std::shared_ptr<cyber::Reader<nav_msgs::Odometry>> sub_odom_aft_mapped;

  nav_msgs::Odometry laser_odometry2;
  tf::StampedTransform laser_odometry_trans2;
  tf::TransformBroadcaster tf_broadcaster2;

  tf::StampedTransform map_2_camera_init_trans;
  tf::TransformBroadcaster tf_broadcaster_map_2_camera_init;

  tf::StampedTransform camera_2_base_link_trans;
  tf::TransformBroadcaster tf_broadcaster_camera_2_baselink;

  Transform transform_sum;
  Transform transform_incre;
  Transform transform_mapped;
  Transform transform_bef_mapped;
  Transform transform_aft_mapped;

  apollo::common::Header current_header;
};

CYBER_REGISTER_COMPONENT(TransformFusion)

}  // namespace tools
}  // namespace apollo
