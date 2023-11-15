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

#include "src/transform_fusion.h"

TransformFusion::TransformFusion() {}

~TransformFusion::TransformFusion() {}

bool TransformFusion::Init() {
  pub_laser_odometry2 = node_->CreateWriter<nav_msgs::Odometry>("/integrated_to_init");
  sub_laser_odometry = node_->CreateReader<nav_msgs::Odometry>(
    FLAGS_laser_odometry_topic,
    [&](const std::shared_ptr<nav_msgs::Odometry>& laser_odometry) {
      LaserOdometryHandler(laser_odometry);
    });

  sub_odom_aft_mapped = node_->CreateReader<nav_msgs::Odometry>(
    FLAGS_odom_aft_mapped_topic,
    [&](const std::shared_ptr<nav_msgs::Odometry>& laser_odometry) {
      OdomAftMappedHandler(laser_odometry);
    });

  laser_odometry2.mutable_header()->set_frame_id("/camera_init");
  laser_odometry2.set_child_frame_id("/camera");

  laser_odometry_trans2.frame_id_ = "/camera_init";
  laser_odometry_trans2.child_frame_id_ = "/camera";

  map_2_camera_init_trans.frame_id_ = "/map";
  map_2_camera_init_trans.child_frame_id_ = "/camera_init";

  camera_2_base_link_trans.frame_id_ = "/camera";
  camera_2_base_link_trans.child_frame_id_ = "/base_link";
}

void TransformFusion::TransformAssociateToMap() {

}

void TransformFusion::LaserOdometryHandler(const std::shared_ptr<nav_msgs::Odometry>& laser_odometry) {

}

void TransformFusion::OdomAftMappedHandler(const std::shared_ptr<nav_msgs::Odometry>& laser_odometry) {

}
