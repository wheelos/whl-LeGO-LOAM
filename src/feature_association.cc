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

#include "src/feature_association.h"

#include <Eigen/Geometry>

#include "src/utility.h"

namespace apollo {
namespace tools {

bool FeatureAssociation::Init() {
  sub_imu_ = node_->CreateReader<>(
    FLAGS_imu_topic,
    [&](const ImuPtr& imu_msg){
      ImuHandler(imu_msg);
  });
  return true;
}

void FeatureAssociation::ImuHandler(const ImuPtr& imu_msg) {
  auto orientation = imu_msg->orientation();
  Eigen::Quaternion<double> q(orientation.w(),
                              orientation.x(),
                              orientation.y(),
                              orientation.z());
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);

  Pose& pose = pose_arr_[imu_pointer_last];

  pose.roll = euler[0];
  pose.yaw  = euler[1];
  pose.pitch = euler[2];

  // pose.linear_acceleration = imu_msg->linear_acceleration();

  pose.angular_velocity[0] = imu_msg->angular_velocity().x();
  pose.angular_velocity[1] = imu_msg->angular_velocity().y();
  pose.angular_velocity[2] = imu_msg->angular_velocity().z();

  imu_pointer_last = (imu_pointer_last + 1) % IMU_QUE_LENGTH;

  AccumulateIMUShiftAndRotation();
}

void FeatureAssociation::AccumulateIMUShiftAndRotation() {
  Pose& pre = pose_arr_[imu_pointer_last - 1];
  Pose& cur = pose_arr_[imu_pointer_last];

  // todo(zero):
  // acc

  double time_interval = cur.timestamp - pre.timestamp;
  if (time_interval < SCAN_PERIOD) {
    cur.position = pre.position + pre.velocity * time_interval + acc * time_interval * time_interval / 2;
    cur.velocity = pre.velocity + acc * time_interval;
    cur.angular = pre.angular + pre.angular_velocity * time_interval;
  }
}

void FeatureAssociation::AdjustDistortion() {

}

void FeatureAssociation::CalculateSmoothness() {
  int cloud_size = segmented_cloud_->points.size();
  for (int i = 5; i < cloud_size - 5; ++i) {
    auto iter = segInfo.segmented_cloud_range.begin() + i;
    auto init_value = -11 * segInfo.segmented_cloud_range[i];
    float diff_range = std::accumulate(iter - 5, iter + 6, init_value);
    cloudCurvature[i] = diff_range * diff_range;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void FeatureAssociation::MarkOccludedPoints() {
  int cloud_size = segmented_cloud_->points.size();
  for (int i = 5; i < cloud_size - 6; ++i) {
    float depth1 = segInfo.segmented_cloud_range[i];
    float depth2 = segInfo.segmented_cloud_range[i + 1];
    int column_diff = std::abs(segInfo.segmented_cloud_col_ind[i+1] -
                          segInfo.segmented_cloud_col_ind[i]);
    if (column_diff < 10) {
      if (depth1 - depth2 > 0.3) {
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
      }
    }

    float diff1 = std::abs(segInfo.segmented_cloud_range[i-1] - segInfo.segmented_cloud_range[i]);
    float diff2 = std::abs(segInfo.segmented_cloud_range[i+1] - segInfo.segmented_cloud_range[i]);
    float threshold = 0.02 * segInfo.segmented_cloud_range[i];
    if (diff1 > threshold && diff2 > threshold) {
      cloudNeighborPicked[i] = 1;
    }
  }
}

void FeatureAssociation::PickCornerPointsSharp(int start, int end) {
  // todo(zero): new function
  int largest_picked_num = 0;
  for (int i = end; i > start; --i) {
    int ind = cloudSmoothness[i].ind;
    if (cloudNeighborPicked[ind] == 0 &&
        cloudCurvature[ind] > edgeThreshold &&
        segInfo.segmented_cloud_ground_flag[ind] == false) {
      ++largest_picked_num;
      if (largest_picked_num <= 2) {
        cloudLabel[ind] = 2;
        cornerPointsSharp->push_back(segmentedCloud->points[ind]);
        cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
      } else if (largest_picked_num <= 20) {
        cloudLabel[ind] = 1;
        cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
      } else {
        break;
      }

      cloudNeighborPicked[ind] = 1;
      for (int j = 1; j <= 5; ++j) {
        int column_diff = std::abs(int(segInfo.segmented_cloud_col_ind[ind + j] - segInfo.segmented_cloud_col_ind[ind + j - 1]));
        if (column_diff > 10)
          break;
        cloudNeighborPicked[ind + j] = 1;
      }

      for (int j = -1; j >= -5; --j) {
        int column_diff = std::abs(int(segInfo.segmentedCloudColInd[ind + j] - segInfo.segmentedCloudColInd[ind + j + 1]));
        if (column_diff > 10)
          break;
        cloudNeighborPicked[ind + j] = 1;
      }
    }
  }
}

void FeatureAssociation::PickSurfPointsFlat(
    int start, int end, int max_pick_num = 4) {
  int smallest_picked_num = 0;
  for (int i = start; i <= end; ++i) {
    int ind = cloudSmoothness[i].ind;
    if (cloudNeighborPicked[ind] == 0 &&
        cloudCurvature[ind] < surfThreshold &&
        segInfo.segmented_cloud_ground_flag[ind] == true) {
      cloudLabel[ind] = -1;
      surfPointsFlat->push_back(segmentedCloud->points[ind]);
      ++smallest_picked_num;
      if (smallest_picked_num >= max_pick_num) {
        break;
      }

      cloudNeighborPicked[ind] = 1;
      for (int j = 1; j <= 5; j++) {
        int column_diff = std::abs(int(segInfo.segmentedCloudColInd[ind + j] - segInfo.segmentedCloudColInd[ind + j - 1]));
        if (column_diff > 10) break;
        cloudNeighborPicked[ind + j] = 1;
      }

      for (int j = -1; j >= -5; j--) {
        int column_diff = std::abs(int(segInfo.segmentedCloudColInd[ind + j] - segInfo.segmentedCloudColInd[ind + j + 1]));
        if (column_diff > 10) break;
        cloudNeighborPicked[ind + j] = 1;
      }
    }
  }
}

void FeatureAssociation::ExtractFeatures() {
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();

  for (int i = 0; i < N_SCAN; ++i) {
    surfPointsLessFlatScan->clear();
    // Divide the circle into 6 equal parts,
    // select 4 surface features and 2 line feature for each direction
    for (int round_id = 0; round_id < 6; ++round_id) {
      int sp = (segInfo.start_ring_index[i] * (6 - round_id) + segInfo.end_ring_index[i] * round_id) / 6;
      int ep = (segInfo.start_ring_index[i] * (5 - round_id) + segInfo.end_ring_index[i] * (round_id + 1)) / 6 - 1;

      if (sp >= ep) continue;

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep,
        [](const smoothness_t& l, const smoothness_t& r){
          return l.value < r.value;
        });

      PickCornerPointsSharp(sp, ep);
      PickSurfPointsFlat(sp, ep, 4);

      for (int k = sp; k <= ep; ++k) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
        }
      }
    }

    surfPointsLessFlatScanDS->clear();
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.filter(*surfPointsLessFlatScanDS);

    *surfPointsLessFlat += *surfPointsLessFlatScanDS;
  }
}

void FeatureAssociation::PublishCloud() {
  ToDriverPointCloud(cornerPointsSharp, laserCloudOutMsg);
  pubCornerPointsSharp.publish(laserCloudOutMsg);

  pubCornerPointsLessSharp.publish(laserCloudOutMsg);
  pubSurfPointsFlat.publish(laserCloudOutMsg);
  pubSurfPointsLessFlat.publish(laserCloudOutMsg);
}

void CheckSystemInitialization(){
  // todo(daohu527):
  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
  kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  sensor_msgs::PointCloud2 laserCloudCornerLast2;
  pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
  laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
  laserCloudCornerLast2.header.frame_id = "/camera";
  pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

  sensor_msgs::PointCloud2 laserCloudSurfLast2;
  pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
  laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
  laserCloudSurfLast2.header.frame_id = "/camera";
  pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

  transformSum[0] += imuPitchStart;
  transformSum[2] += imuRollStart;

  systemInitedLM = true;
}

void UpdateInitialGuess() {
  // todo(daohu527):
  imuPitchLast = imuPitchCur;
  imuYawLast = imuYawCur;
  imuRollLast = imuRollCur;

  imuShiftFromStartX = imuShiftFromStartXCur;
  imuShiftFromStartY = imuShiftFromStartYCur;
  imuShiftFromStartZ = imuShiftFromStartZCur;

  imuVeloFromStartX = imuVeloFromStartXCur;
  imuVeloFromStartY = imuVeloFromStartYCur;
  imuVeloFromStartZ = imuVeloFromStartZCur;

  if (imuAngularFromStartX != 0 || imuAngularFromStartY != 0 || imuAngularFromStartZ != 0){
    transformCur[0] = - imuAngularFromStartY;
    transformCur[1] = - imuAngularFromStartZ;
    transformCur[2] = - imuAngularFromStartX;
  }

  if (imuVeloFromStartX != 0 || imuVeloFromStartY != 0 || imuVeloFromStartZ != 0){
    transformCur[3] -= imuVeloFromStartX * scanPeriod;
    transformCur[4] -= imuVeloFromStartY * scanPeriod;
    transformCur[5] -= imuVeloFromStartZ * scanPeriod;
  }
}

void UpdateTransformation() {
  if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100)
    return;

  for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingSurfFeatures(iterCount1);

    if (laserCloudOri->points.size() < 10)
      continue;
    if (calculateTransformationSurf(iterCount1) == false)
      break;
  }

  for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {
    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingCornerFeatures(iterCount2);

    if (laserCloudOri->points.size() < 10)
      continue;
    if (calculateTransformationCorner(iterCount2) == false)
      break;
  }
}

void IntegrateTransformation() {
  float rx, ry, rz, tx, ty, tz;
  AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
                     -transformCur[0], -transformCur[1], -transformCur[2], rx, ry, rz);

  float x1 = cos(rz) * (transformCur[3] - imuShiftFromStartX) - sin(rz) * (transformCur[4] - imuShiftFromStartY);
  float y1 = sin(rz) * (transformCur[3] - imuShiftFromStartX) + cos(rz) * (transformCur[4] - imuShiftFromStartY);
  float z1 = transformCur[5] - imuShiftFromStartZ;

  float x2 = x1;
  float y2 = cos(rx) * y1 - sin(rx) * z1;
  float z2 = sin(rx) * y1 + cos(rx) * z1;

  tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
  ty = transformSum[4] - y2;
  tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

  PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart,
                    imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

  transformSum[0] = rx;
  transformSum[1] = ry;
  transformSum[2] = rz;
  transformSum[3] = tx;
  transformSum[4] = ty;
  transformSum[5] = tz;
}

void PublishOdometry() {
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);

  laserOdometry.header.stamp = cloudHeader.stamp;
  laserOdometry.pose.pose.orientation.x = -geoQuat.y;
  laserOdometry.pose.pose.orientation.y = -geoQuat.z;
  laserOdometry.pose.pose.orientation.z = geoQuat.x;
  laserOdometry.pose.pose.orientation.w = geoQuat.w;
  laserOdometry.pose.pose.position.x = transformSum[3];
  laserOdometry.pose.pose.position.y = transformSum[4];
  laserOdometry.pose.pose.position.z = transformSum[5];
  pubLaserOdometry.publish(laserOdometry);

  laserOdometryTrans.stamp_ = cloudHeader.stamp;
  laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
  tfBroadcaster.sendTransform(laserOdometryTrans);
}

void PublishCloudsLast() {
  updateImuRollPitchYawStartSinCos();

  int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
  for (int i = 0; i < cornerPointsLessSharpNum; i++) {
    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
  }

  int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
  for (int i = 0; i < surfPointsLessFlatNum; i++) {
    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
  }

  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
  }

  frameCount++;

  if (frameCount >= skipFrameNum + 1) {
    frameCount = 0;

    adjustOutlierCloud();
    sensor_msgs::PointCloud2 outlierCloudLast2;
    pcl::toROSMsg(*outlierCloud, outlierCloudLast2);
    outlierCloudLast2.header.stamp = cloudHeader.stamp;
    outlierCloudLast2.header.frame_id = "/camera";
    pubOutlierCloudLast.publish(outlierCloudLast2);

    sensor_msgs::PointCloud2 laserCloudCornerLast2;
    pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
    laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
    laserCloudCornerLast2.header.frame_id = "/camera";
    pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

    sensor_msgs::PointCloud2 laserCloudSurfLast2;
    pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
    laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
    laserCloudSurfLast2.header.frame_id = "/camera";
    pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
  }
}

void FeatureAssociation::RunFeatureAssociation() {
  // 1. Feature Extraction
  AdjustDistortion();

  CalculateSmoothness();

  MarkOccludedPoints();

  ExtractFeatures();

  PublishCloud();


  if (!systemInitedLM) {
    CheckSystemInitialization();
    return;
  }

  UpdateInitialGuess();

  UpdateTransformation();

  IntegrateTransformation();

  PublishOdometry();

  // cloud to mapOptimization
  PublishCloudsLast();
}

}  // namespace tools
}  // namespace apollo
