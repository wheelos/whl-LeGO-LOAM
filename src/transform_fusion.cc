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

namespace apollo {
namespace tools {

bool TransformFusion::Init() {
  sub_laser_odometry = node_->CreateReader<nav_msgs::Odometry>(
    "/laser_odom_to_init",
    [](const nav_msgs::Odometry::ConstPtr &msg){
      LaserOdometryHandler(msg);
    });

  sub_odom_aft_mapped = node_->CreateReader<nav_msgs::Odometry>(
    "/aft_mapped_to_init",
    [](const nav_msgs::Odometry::ConstPtr &msg){
      OdomAftMappedHandler(msg);
    });
  return true;
}

void TransformFusion::TransformAssociateToMap() {
  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

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

  float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz) - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
  transformMapped[0] = -asin(srx);

  float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx) - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly) + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
  float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx) + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly) - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);
  transformMapped[1] = atan2(srycrx / cos(transformMapped[0]),
                              crycrx / cos(transformMapped[0]));

  float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]),
                              crzcrx / cos(transformMapped[0]));

  x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
  y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
  z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

  transformMapped[3] = transformAftMapped[3] - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
  transformMapped[4] = transformAftMapped[4] - y2;
  transformMapped[5] = transformAftMapped[5] - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
}

void TransformFusion::LaserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformSum[0] = -pitch;
  transformSum[1] = -yaw;
  transformSum[2] = roll;

  transformSum[3] = laserOdometry->pose.pose.position.x;
  transformSum[4] = laserOdometry->pose.pose.position.y;
  transformSum[5] = laserOdometry->pose.pose.position.z;

  TransformAssociateToMap();

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformMapped[2], -transformMapped[0], -transformMapped[1]);

  laserOdometry2.header.stamp = laserOdometry->header.stamp;
  laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
  laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
  laserOdometry2.pose.pose.orientation.z = geoQuat.x;
  laserOdometry2.pose.pose.orientation.w = geoQuat.w;
  laserOdometry2.pose.pose.position.x = transformMapped[3];
  laserOdometry2.pose.pose.position.y = transformMapped[4];
  laserOdometry2.pose.pose.position.z = transformMapped[5];
  pubLaserOdometry2.publish(laserOdometry2);

  laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
  laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
  tfBroadcaster2.sendTransform(laserOdometryTrans2);
}

void TransformFusion::OdomAftMappedHandler(const nav_msgs::Odometry::ConstPtr &odomAftMapped) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
  // Eigen::Quaternion<double> q(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w);
  // Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);

  transformAftMapped[0] = -pitch;
  transformAftMapped[1] = -yaw;
  transformAftMapped[2] = roll;

  transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
  transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
  transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

  transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
  transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
  transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

  transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
  transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
  transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
}

}  // namespace tools
}  // namespace apollo
