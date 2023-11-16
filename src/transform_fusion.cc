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
  // transformSum * (transformBefMapped - transformSum) -> transformIncre
  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
            - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
            + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

  // pitch, yaw, roll -> transformMapped
  float sbcx = sin(transformSum[0]);
  float cbcx = cos(transformSum[0]);
  float sbcy = sin(transformSum[1]);
  float cbcy = cos(transformSum[1]);
  float sbcz = sin(transformSum[2]);
  float cbcz = cos(transformSum[2]);

  float sblx = sin(transformBefMapped[0]);
  float cblx = cos(transformBefMapped[0]);
  float sbly = sin(transformBefMapped[1]);
  float cbly = cos(transformBefMapped[1]);
  float sblz = sin(transformBefMapped[2]);
  float cblz = cos(transformBefMapped[2]);

  float salx = sin(transformAftMapped[0]);
  float calx = cos(transformAftMapped[0]);
  float saly = sin(transformAftMapped[1]);
  float caly = cos(transformAftMapped[1]);
  float salz = sin(transformAftMapped[2]);
  float calz = cos(transformAftMapped[2]);

  float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
            - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
            - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
            - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
            - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
  transformMapped[0] = -asin(srx);

  float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
  float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
  transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), 
                              crycrx / cos(transformMapped[0]));
  
  float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), 
                              crzcrx / cos(transformMapped[0]));

  // transformIncre is [x,y,z] * roll * pitch
  x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
  y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
  z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

  // transformAftMapped is [x,y,z], + increase * yaw
  transformMapped[3] = transformAftMapped[3] 
                      - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
  transformMapped[4] = transformAftMapped[4] - y2;
  transformMapped[5] = transformAftMapped[5] 
                      - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
}

void TransformFusion::LaserOdometryHandler(const std::shared_ptr<nav_msgs::Odometry>& laser_odometry) {
  current_header = laser_odometry->header;
  geometry_msgs::Quaternion geo_quat = laser_odometry->pose.pose.orientation;
  Eigen::Quaterniond q(geo_quat.z, -geo_quat.x, -geo_quat.y, geo_quat.w);
  auto [yaw, roll, pitch] = q.toRotationMatrix().eulerAngles(0, 1, 2);
  transform_sum.rotation[0] = -pitch;
  transform_sum.rotation[1] = -yaw;
  transform_sum.rotation[2] = roll;

  transform_sum.translation[0] = laser_odometry->pose.pose.position.x;
  transform_sum.translation[1] = laser_odometry->pose.pose.position.y;
  transform_sum.translation[2] = laser_odometry->pose.pose.position.z;

  TransformAssociateToMap();

  // todo(zero): why need euler transform???
  laser_odometry2.mutable_header()->set_timestamp(laser_odometry->header().timestamp());
  laser_odometry2.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(transform_mapped.rotation[0]);
  laser_odometry2.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(transform_mapped.rotation[1]);
  laser_odometry2.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(transform_mapped.rotation[2]);
  laser_odometry2.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(transform_mapped.rotation[3]);

  laser_odometry2.mutable_pose()->mutable_pose()->mutable_position()->set_x(transform_mapped.translation[0]);
  laser_odometry2.mutable_pose()->mutable_pose()->mutable_position()->set_y(transform_mapped.translation[1]);
  laser_odometry2.mutable_pose()->mutable_pose()->mutable_position()->set_z(transform_mapped.translation[2]);

  pub_laser_odometry2->Write(laser_odometry2);

  laser_odometry_trans2.stamp_ = laser_odometry->header.stamp;
  laser_odometry_trans2.setRotation();
  laser_odometry_trans2.setOrigin(transform_mapped.translation);
  tf_broadcaster2.sendTransform(laser_odometry_trans2);
}

void TransformFusion::OdomAftMappedHandler(const std::shared_ptr<nav_msgs::Odometry>& odom_aft_mapped) {
  // todo(zero): why need euler transform???
  geometry_msgs::Quaternion geo_quat = odom_aft_mapped->pose.pose.orientation;
  Eigen::Quaterniond q(geo_quat.z, -geo_quat.x, -geo_quat.y, geo_quat.w);
  auto [yaw, roll, pitch] = q.toRotationMatrix().eulerAngles(0, 1, 2);
  transform_aft_mapped.rotation[0] = -pitch;
  transform_aft_mapped.rotation[1] = -yaw;
  transform_aft_mapped.rotation[2] = roll;

  transform_aft_mapped.translation[0] = odom_aft_mapped->pose.pose.position.x;
  transform_aft_mapped.translation[1] = odom_aft_mapped->pose.pose.position.y;
  transform_aft_mapped.translation[2] = odom_aft_mapped->pose.pose.position.z;

  transform_bef_mapped.rotation[0] = odom_aft_mapped->twist.twist.angular.x;
  transform_bef_mapped.rotation[1] = odom_aft_mapped->twist.twist.angular.y;
  transform_bef_mapped.rotation[2] = odom_aft_mapped->twist.twist.angular.z;

  transform_aft_mapped.translation[0] = odom_aft_mapped->twist.twist.linear.x;
  transform_aft_mapped.translation[1] = odom_aft_mapped->twist.twist.linear.y;
  transform_aft_mapped.translation[2] = odom_aft_mapped->twist.twist.linear.z;
}
