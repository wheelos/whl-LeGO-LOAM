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

void ShiftToStartIMU() {
  imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
  imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
  imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

  float x1 = cosImuYawStart * imuShiftFromStartXCur - sinImuYawStart * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sinImuYawStart * imuShiftFromStartXCur + cosImuYawStart * imuShiftFromStartZCur;

  float x2 = x1;
  float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
  float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

  imuShiftFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
  imuShiftFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
  imuShiftFromStartZCur = z2;
}

void VeloToStartIMU() {
  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

  float x1 = cosImuYawStart * imuVeloFromStartXCur - sinImuYawStart * imuVeloFromStartZCur;
  float y1 = imuVeloFromStartYCur;
  float z1 = sinImuYawStart * imuVeloFromStartXCur + cosImuYawStart * imuVeloFromStartZCur;

  float x2 = x1;
  float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
  float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

  imuVeloFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
  imuVeloFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
  imuVeloFromStartZCur = z2;
}

void TransformToStartIMU() {
  float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
  float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
  float z1 = p->z;

  float x2 = x1;
  float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
  float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

  float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
  float y3 = y2;
  float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

  float x4 = cosImuYawStart * x3 - sinImuYawStart * z3;
  float y4 = y3;
  float z4 = sinImuYawStart * x3 + cosImuYawStart * z3;

  float x5 = x4;
  float y5 = cosImuPitchStart * y4 + sinImuPitchStart * z4;
  float z5 = -sinImuPitchStart * y4 + cosImuPitchStart * z4;

  p->x = cosImuRollStart * x5 + sinImuRollStart * y5 + imuShiftFromStartXCur;
  p->y = -sinImuRollStart * x5 + cosImuRollStart * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
}

void AccumulateIMUShiftAndRotation() {
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;

  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (timeDiff < scanPeriod)
  {

    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;

    imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
    imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
    imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
  }
}


void FeatureAssociation::ImuHandler(
    const std::shared_ptr<apollo::localization::CorrectedImu>& imu_in) {
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
  float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
  float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();

  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw;

  imuAccX[imuPointerLast] = accX;
  imuAccY[imuPointerLast] = accY;
  imuAccZ[imuPointerLast] = accZ;

  imuAngularVeloX[imuPointerLast] = imuIn->angular_velocity.x;
  imuAngularVeloY[imuPointerLast] = imuIn->angular_velocity.y;
  imuAngularVeloZ[imuPointerLast] = imuIn->angular_velocity.z;

  AccumulateIMUShiftAndRotation();
}

void FeatureAssociation::LaserCloudHandler() {
  cloudHeader = laserCloudMsg->header;

  timeScanCur = cloudHeader.stamp.toSec();
  timeNewSegmentedCloud = timeScanCur;

  segmentedCloud->clear();
  pcl::fromROSMsg(*laserCloudMsg, *segmentedCloud);

  newSegmentedCloud = true;
}

void FeatureAssociation::OutlierCloudHandler() {
  timeNewOutlierCloud = msgIn->header.stamp.toSec();

  outlierCloud->clear();
  pcl::fromROSMsg(*msgIn, *outlierCloud);

  newOutlierCloud = true;
}

void FeatureAssociation::LaserCloudInfoHandler() {
  timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
  segInfo = *msgIn;
  newSegmentedCloudInfo = true;
}


bool FeatureAssociation::Proc() {
  // todo(zero): check newSegmentedCloud, newSegmentedCloudInfo, newOutlierCloud
  // timediff < 0.05

  // 1. Feature Extraction
  AdjustDistortion();

  CalculateSmoothness();

  MarkOccludedPoints();

  ExtractFeatures();

  PublishCloud();


  // 2. Feature Association
  if (!system_inited_LM) {
    CheckSystemInitialization();
    return;
  }

  UpdateInitialGuess();

  UpdateTransformation();

  IntegrateTransformation();

  PublishOdometry();

  PublishCloudsLast(); // cloud to mapOptimization

  return true;
}


void FeatureAssociation::AdjustDistortion() {
  // todo(zero): need refactor
}

void FeatureAssociation::CalculateSmoothness() {
  int cloud_size = segmented_cloud->points.size();
  for (int i = 5; i < cloud_size - 5; ++i) {
    auto cur = seg_info.segmented_cloud_range.begin() + i;

    float diff_range = std::accumulate(cur - 5, cur + 5, -11*(*cur));

    cloud_curvature[i] = diff_range * diff_range;
    cloud_neighbor_picked[i] = 0;
    cloud_label[i] = 0;

    cloud_smoothness[i].value = cloud_curvature[i];
    cloud_smoothness[i].ind = i;
  }
}


void FeatureAssociation::MarkOccludedPoints() {
  int cloud_size = segmented_cloud->points.size();
  for (int i = 5; i < cloud_size - 6; ++i) {
    auto range_iter = seg_info.segmented_cloud_range.begin() + i;

    float depth1 = *range_iter;
    float depth2 = *(range_iter + 1);
    int column_diff = std::abs(static_cast<int>(
                        seg_info.segmented_cloud_col_ind(i+1) -
                        seg_info.segmented_cloud_col_ind(i)));
    if (column_diff < 10) {
      if (depth1 - depth2 > 0.3) {
        memset(cloud_neighbor_picked + i - 5, 1, 6 * sizeof(int));
      } else if (depth2 - depth1 > 0.3) {
        memset(cloud_neighbor_picked + i + 1, 1, 6 * sizeof(int));
      }
    }

    float diff1 = std::fabs(*(range_iter - 1) - *range_iter);
    float diff2 = std::fabs(*(range_iter + 1) - *range_iter);

    int threshold = 0.02 * (*range_iter);
    if (diff1 > threshold && diff2 > threshold) {
      cloud_neighbor_picked[i] = 1;
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
    // 360 degrees divided into 6 equal parts
    for (int j = 0; j < 6; ++j) {
      // todo(zero): refactor
      int sp = (seg_info.start_ring_index(i) * (6 - j) + seg_info.end_ring_index(i) * j) / 6;
      int ep = (seg_info.start_ring_index(i) * (5 - j) + seg_info.end_ring_index(i) * (j + 1)) / 6 - 1;

      if (sp >= ep)
        continue;

      // Sort curvature
      std::sort(cloud_smoothness.begin() + sp, cloud_smoothness.begin() + ep, by_value());

      // line feature
      int largest_picked_num = 0;
      for (int k = ep; k >= sp; --k) {
        int ind = cloud_smoothness[k].ind;
        if (cloud_neighbor_picked[ind] == 0 &&
            cloud_curvature[ind] > edgeThreshold &&
            !seg_info.segmented_cloud_ground_flag(ind)) {

          ++largest_picked_num;
          if (largest_picked_num <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(segmented_cloud->points[ind]);
            cornerPointsLessSharp->push_back(segmented_cloud->points[ind]);
          } else if (largest_picked_num <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(segmented_cloud->points[ind]);
          } else {
            break;
          }

          cloud_neighbor_picked[ind] = 1;
          auto iter = seg_info.segmented_cloud_col_ind.begin() + ind;
          for (int l = 1; l <= 5; ++l) {
            int column_diff = std::abs(*(iter + l) - *(iter + l - 1));
            if (column_diff > 10)
              break;
            cloud_neighbor_picked[ind + l] = 1;
          }

          for (int l = -1; l >= -5; --l) {
            int column_diff = std::abs(*(iter + l) - *(iter + l + 1));
            if (column_diff > 10)
              break;
            cloud_neighbor_picked[ind + l] = 1;
          }
        }
      }

      // flat feature
      int smallest_picked_num = 0;
      for (int k = sp; k <= ep; ++k) {
        int ind = cloud_smoothness[k].ind;
        if (cloud_neighbor_picked[ind] == 0 &&
            cloud_curvature[ind] < surfThreshold &&
            seg_info.segmented_cloud_ground_flag(ind)) {
          cloud_label[ind] = -1;
          surfPointsFlat->push_back(segmented_cloud->points[ind]);

          ++smallest_picked_num;
          if (smallest_picked_num >= 4) {
            break;
          }

          cloud_neighbor_picked[ind] = 1;
          auto iter = seg_info.segmented_cloud_col_ind.begin() + ind;
          for (int l = 1; l <= 5; ++l) {
            int column_diff = std::abs(*(iter + l) - *(iter + l - 1));
            if (column_diff > 10)
              break;
            cloud_neighbor_picked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; --l) {
            int column_diff = std::abs(*(iter + l) - *(iter + l + 1));
            if (column_diff > 10)
              break;
            cloud_neighbor_picked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; ++k) {
        if (cloud_label[k] <= 0) {
          surfPointsLessFlatScan->push_back(segmented_cloud->points[k]);
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
  apollo::drivers::PointCloud laserCloudOutMsg;
  ToDriverPointCloud(cornerPointsSharp, laserCloudOutMsg);
  pub_corner_points_sharp->Write(laserCloudOutMsg);

  laserCloudOutMsg.mutable_point()->Clear();
  ToDriverPointCloud(cornerPointsLessSharp, laserCloudOutMsg);
  pub_corner_points_less_sharp->Write(laserCloudOutMsg);

  laserCloudOutMsg.mutable_point()->Clear();
  ToDriverPointCloud(surfPointsFlat, laserCloudOutMsg);
  pub_surf_points_flat->Write(laserCloudOutMsg);

  laserCloudOutMsg.mutable_point()->Clear();
  ToDriverPointCloud(surfPointsLessFlat, laserCloudOutMsg);
  pub_surf_points_less_flat->Write(laserCloudOutMsg);
}

void FeatureAssociation::CheckSystemInitialization() {
  // todo(zero): why need swap???
  std::swap(cornerPointsLessSharp, laserCloudCornerLast);
  std::swap(surfPointsLessFlat, laserCloudSurfLast);

  kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
  kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laerCloudSurfLastNum = laserCloudSurfLast->points.size();

  // Publish laserCloudCornerLast
  apollo::drivers::PointCloud laserCloudCornerLast2;
  ToDriverPointCloud(laserCloudCornerLast, laserCloudCornerLast2);
  pub_laser_cloud_corner_last->Write(laserCloudCornerLast2);

  // Publish laserCloudSurfLast
  apollo::drivers::PointCloud laserCloudSurfLast2;
  ToDriverPointCloud(laserCloudSurfLast, laserCloudSurfLast2);
  pub_laser_cloud_surf_last->Write(laserCloudSurfLast2);

  // Init transformSum
  transformSum[0] += imuPitchStart;
  transformSum[2] += imuRollStart;

  system_inited_LM = true;
}

void FeatureAssociation::UpdateInitialGuess() {
  imuPitchLast = imuPitchCur;
  imuYawLast = imuYawCur;
  imuRollLast = imuRollCur;

  imuShiftFromStartX = imuShiftFromStartXCur;
  imuShiftFromStartY = imuShiftFromStartYCur;
  imuShiftFromStartZ = imuShiftFromStartXCur;

  imuVeloFromStartX = imuVeloFromStartXCur;
  imuVeloFromStartY = imuVeloFromStartYCur;
  imuVeloFromStartZ = imuVeloFromStartZCur;

  // todo(zero): imu coor -> ???
  if (imuAngularFromStartX || imuAngularFromStartY || imuAngularFromStartZ) {
    transformCur[0] = -imuAngularFromStartY;
    transformCur[1] = -imuAngularFromStartZ;
    transformCur[2] = -imuAngularFromStartX;
  }

  if (imuVeloFromStartX || imuVeloFromStartY || imuVeloFromStartZ) {
    transformCur[3] -= imuVeloFromStartX * scanPeriod;
    transformCur[4] -= imuVeloFromStartY * scanPeriod;
    transformCur[5] -= imuVeloFromStartZ * scanPeriod;
  }
}

void FeatureAssociation::UpdateTransformation() {
  if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100)
    return;

  for (int iterCount = 0; iterCount < 25; ++iterCount) {
    laserCloudOri->clear();
    coeffSel->clear();

    FindCorrespondingSurfFeatures(iterCount);

    if (laserCloudOri->points.size() < 10)
      continue;
    if (!CalculateTransformationSurf(iterCount))
      break;
  }

  for (int iterCount = 0; iterCount < 25; ++iterCount) {
    laserCloudOri->clear();
    coeffSel->clear();

    FindCorrespondingCornerFeatures(iterCount);

    if (laserCloudOri->points.size() < 10)
      continue;
    if (!CalculateTransformationCorner(iterCount))
      break;
  }
}

void FeatureAssociation::TransformToStart(const PointType& pi, PointType& po) {
  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transformCur[0];
  float ry = s * transformCur[1];
  float rz = s * transformCur[2];
  float tx = s * transformCur[3];
  float ty = s * transformCur[4];
  float tz = s * transformCur[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  po->intensity = pi->intensity;
}

void FeatureAssociation::FindCorrespondingSurfFeatures(int iterCount) {
  int surfPointsFlatNum = surfPointsFlat->points.size();

  for (int i = 0; i < surfPointsFlatNum; ++i) {
    TransformToStart(&surfPointsFlat->points[i], &pointSel);

    if (iterCount % 5 == 0) {
      kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
      int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

      if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
        for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
          if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                           (laserCloudSurfLast->points[j].x - pointSel.x) +
                       (laserCloudSurfLast->points[j].y - pointSel.y) *
                           (laserCloudSurfLast->points[j].y - pointSel.y) +
                       (laserCloudSurfLast->points[j].z - pointSel.z) *
                           (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                           (laserCloudSurfLast->points[j].x - pointSel.x) +
                       (laserCloudSurfLast->points[j].y - pointSel.y) *
                           (laserCloudSurfLast->points[j].y - pointSel.y) +
                       (laserCloudSurfLast->points[j].z - pointSel.z) *
                           (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
      }

      pointSearchSurfInd1[i] = closestPointInd;
      pointSearchSurfInd2[i] = minPointInd2;
      pointSearchSurfInd3[i] = minPointInd3;
    }

    if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
      tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
      tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
      tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

      float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
      float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
      float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
      float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

      float ps = sqrt(pa * pa + pb * pb + pc * pc);

      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
      }

      if (s > 0.1 && pd2 != 0) {
        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        laserCloudOri->push_back(surfPointsFlat->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

bool FeatureAssociation::CalculateTransformationSurf(int iterCount) {
  int pointSelNum = laserCloudOri->points.size();

  cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float a1 = crx * sry * srz;
  float a2 = crx * crz * sry;
  float a3 = srx * sry;
  float a4 = tx * a1 - ty * a2 - tz * a3;
  float a5 = srx * srz;
  float a6 = crz * srx;
  float a7 = ty * a6 - tz * crx - tx * a5;
  float a8 = crx * cry * srz;
  float a9 = crx * cry * crz;
  float a10 = cry * srx;
  float a11 = tz * a10 + ty * a9 - tx * a8;

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;

  float c1 = -b6;
  float c2 = b5;
  float c3 = tx * b6 - ty * b5;
  float c4 = -crx * crz;
  float c5 = crx * srz;
  float c6 = ty * c5 + tx * -c4;
  float c7 = b2;
  float c8 = -b1;
  float c9 = tx * -b2 - ty * -b1;

  for (int i = 0; i < pointSelNum; i++) {
    pointOri = laserCloudOri->points[i];
    coeff = coeffSel->points[i];

    float arx = (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x + (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y + (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

    float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x + (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y + (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

    float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

    float d2 = coeff.intensity;

    matA.at<float>(i, 0) = arx;
    matA.at<float>(i, 1) = arz;
    matA.at<float>(i, 2) = aty;
    matB.at<float>(i, 0) = -0.05 * d2;
  }

  cv::transpose(matA, matAt);
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

  if (iterCount == 0) {
    cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

    cv::eigen(matAtA, matE, matV);
    matV.copyTo(matV2);

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE.at<float>(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
          matV2.at<float>(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inv() * matV2;
  }

  if (isDegenerate) {
    cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP * matX2;
  }

  transformCur[0] += matX.at<float>(0, 0);
  transformCur[2] += matX.at<float>(1, 0);
  transformCur[4] += matX.at<float>(2, 0);

  for (int i = 0; i < 6; i++) {
    if (isnan(transformCur[i]))
      transformCur[i] = 0;
  }

  float deltaR = sqrt(
      pow(rad2deg(matX.at<float>(0, 0)), 2) +
      pow(rad2deg(matX.at<float>(1, 0)), 2));
  float deltaT = sqrt(
      pow(matX.at<float>(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

void FeatureAssociation::FindCorrespondingCornerFeatures(int iterCount) {
  int cornerPointsSharpNum = cornerPointsSharp->points.size();
  for (int i = 0; i < cornerPointsSharpNum; ++i) {
    TransformToStart(&cornerPointsSharp->points[i], &pointSel);

    if (iterCount % 5 == 0) {
      kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
      int closestPointInd = -1, minPointInd2 = -1;

      if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
        for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
          if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                           (laserCloudCornerLast->points[j].x - pointSel.x) +
                       (laserCloudCornerLast->points[j].y - pointSel.y) *
                           (laserCloudCornerLast->points[j].y - pointSel.y) +
                       (laserCloudCornerLast->points[j].z - pointSel.z) *
                           (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }

        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                           (laserCloudCornerLast->points[j].x - pointSel.x) +
                       (laserCloudCornerLast->points[j].y - pointSel.y) *
                           (laserCloudCornerLast->points[j].y - pointSel.y) +
                       (laserCloudCornerLast->points[j].z - pointSel.z) *
                           (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
      }

      pointSearchCornerInd1[i] = closestPointInd;
      pointSearchCornerInd2[i] = minPointInd2;
    }

    if (pointSearchCornerInd2[i] >= 0) {
      tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
      tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

      float x0 = pointSel.x;
      float y0 = pointSel.y;
      float z0 = pointSel.z;
      float x1 = tripod1.x;
      float y1 = tripod1.y;
      float z1 = tripod1.z;
      float x2 = tripod2.x;
      float y2 = tripod2.y;
      float z2 = tripod2.z;

      float m11 = ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1));
      float m22 = ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1));
      float m33 = ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1));

      float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

      float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

      float la = ((y1 - y2) * m11 + (z1 - z2) * m22) / a012 / l12;

      float lb = -((x1 - x2) * m11 - (z1 - z2) * m33) / a012 / l12;

      float lc = -((x1 - x2) * m22 + (y1 - y2) * m33) / a012 / l12;

      float ld2 = a012 / l12;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 - 1.8 * std::fabs(ld2);
      }

      if (s > 0.1 && ld2 != 0) {
        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        laserCloudOri->push_back(cornerPointsSharp->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

bool FeatureAssociation::CalculateTransformationCorner(int iterCount) {
  int pointSelNum = laserCloudOri->points.size();

  cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b3 = crx * cry;
  float b4 = tx * -b1 + ty * -b2 + tz * b3;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;
  float b7 = crx * sry;
  float b8 = tz * b7 - ty * b6 - tx * b5;

  float c5 = crx * srz;

  for (int i = 0; i < pointSelNum; i++) {
    pointOri = laserCloudOri->points[i];
    coeff = coeffSel->points[i];

    float ary = (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x + (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

    float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

    float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

    float d2 = coeff.intensity;

    matA.at<float>(i, 0) = ary;
    matA.at<float>(i, 1) = atx;
    matA.at<float>(i, 2) = atz;
    matB.at<float>(i, 0) = -0.05 * d2;
  }

  cv::transpose(matA, matAt);
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

  if (iterCount == 0) {
    cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

    cv::eigen(matAtA, matE, matV);
    matV.copyTo(matV2);

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE.at<float>(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
          matV2.at<float>(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inv() * matV2;
  }

  if (isDegenerate) {
    cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
    matX.copyTo(matX2);
    matX = matP * matX2;
  }

  transformCur[1] += matX.at<float>(0, 0);
  transformCur[3] += matX.at<float>(1, 0);
  transformCur[5] += matX.at<float>(2, 0);

  for (int i = 0; i < 6; i++) {
    if (isnan(transformCur[i]))
      transformCur[i] = 0;
  }

  float deltaR = sqrt(
      pow(rad2deg(matX.at<float>(0, 0)), 2));
  float deltaT = sqrt(
      pow(matX.at<float>(1, 0) * 100, 2) +
      pow(matX.at<float>(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

void FeatureAssociation::AccumulateRotation(float cx, float cy, float cz,
    float lx, float ly, float lz, float &ox, float &oy, float &oz) {
  float srx = cos(lx) * cos(cx) * sin(ly) * sin(cz) - cos(cx) * cos(cz) * sin(lx) - cos(lx) * cos(ly) * sin(cx);
  ox = -asin(srx);

  float srycrx = sin(lx) * (cos(cy) * sin(cz) - cos(cz) * sin(cx) * sin(cy)) + cos(lx) * sin(ly) * (cos(cy) * cos(cz) + sin(cx) * sin(cy) * sin(cz)) + cos(lx) * cos(ly) * cos(cx) * sin(cy);
  float crycrx = cos(lx) * cos(ly) * cos(cx) * cos(cy) - cos(lx) * sin(ly) * (cos(cz) * sin(cy) - cos(cy) * sin(cx) * sin(cz)) - sin(lx) * (sin(cy) * sin(cz) + cos(cy) * cos(cz) * sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx = sin(cx) * (cos(lz) * sin(ly) - cos(ly) * sin(lx) * sin(lz)) + cos(cx) * sin(cz) * (cos(ly) * cos(lz) + sin(lx) * sin(ly) * sin(lz)) + cos(lx) * cos(cx) * cos(cz) * sin(lz);
  float crzcrx = cos(lx) * cos(lz) * cos(cx) * cos(cz) - cos(cx) * sin(cz) * (cos(ly) * sin(lz) - cos(lz) * sin(lx) * sin(ly)) - sin(cx) * (sin(ly) * sin(lz) + cos(ly) * cos(lz) * sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void FeatureAssociation::PluginIMURotation(float bcx, float bcy, float bcz,
    float blx, float bly, float blz, float alx, float aly, float alz,
    float &acx, float &acy, float &acz) {
  float sbcx = sin(bcx);
  float cbcx = cos(bcx);
  float sbcy = sin(bcy);
  float cbcy = cos(bcy);
  float sbcz = sin(bcz);
  float cbcz = cos(bcz);

  float sblx = sin(blx);
  float cblx = cos(blx);
  float sbly = sin(bly);
  float cbly = cos(bly);
  float sblz = sin(blz);
  float cblz = cos(blz);

  float salx = sin(alx);
  float calx = cos(alx);
  float saly = sin(aly);
  float caly = cos(aly);
  float salz = sin(alz);
  float calz = cos(alz);

  float srx = -sbcx * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly) - cbcx * cbcz * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) - cbcx * sbcz * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz);
  acx = -asin(srx);

  float srycrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz) + cbcx * sbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
  float crycrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) + cbcx * cbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
  acy = atan2(srycrx / cos(acx), crycrx / cos(acx));

  float srzcrx = sbcx * (cblx * cbly * (calz * saly - caly * salx * salz) - cblx * sbly * (caly * calz + salx * saly * salz) + calx * salz * sblx) - cbcx * cbcz * ((caly * calz + salx * saly * salz) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cblz * salz) + cbcx * sbcz * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * cblx * salz * sblz);
  float crzcrx = sbcx * (cblx * sbly * (caly * salz - calz * salx * saly) - cblx * cbly * (saly * salz + caly * calz * salx) + calx * calz * sblx) + cbcx * cbcz * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * calz * cblx * cblz) - cbcx * sbcz * ((saly * salz + caly * calz * salx) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (cbly * cblz + sblx * sbly * sblz) - calx * calz * cblx * sblz);
  acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
}

void FeatureAssociation::IntegrateTransformation() {
  float rx, ry, rz, tx, ty, tz;
  AccumulateRotation(transformSum[0], transformSum[1], transfromSum[2],
      -transformCur[0], -transformCur[1], -transformCur[2], rx, ry, rz);

  float x1 = cos(rz) * (transformCur[3] - imuShiftFromStartX)
           - sin(rz) * (transformCur[4] - imuShiftFromStartY);
  float y1 = sin(rz) * (transformCur[3] - imuShiftFromStartX)
           + cos(rz) * (transformCur[4] - imuShiftFromStartY);
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

void FeatureAssociation::PublishOdometry() {
  Eigen::Quaterniond geo_quat = AngleAxisf(transformSum[2], Vector3f::UnitX()) *
                                AngleAxisf(-transformSum[0], Vector3f::UnitY()) *
                                AngleAxisf(-transformSum[1], Vector3f::UnitZ()));
  laserOdometry.header.stamp = cloud_header.stamp;
  // todo(zero): why need transform???
  laserOdometry.pose.pose.orientation.x = -geo_quat.y;
  laserOdometry.pose.pose.orientation.y = -geo_quat.z;
  laserOdometry.pose.pose.orientation.z = geo_quat.x;
  laserOdometry.pose.pose.orientation.w = geo_quat.w;
  laserOdometry.pose.pose.position.x = transformSum[3];
  laserOdometry.pose.pose.position.y = transformSum[4];
  laserOdometry.pose.pose.position.z = transformSum[5];

  pub_laser_odometry->Write(laserOdometry);

  laserOdometryTrans.stamp_ = cloud_header.stamp;
  laserOdometryTrans.setRotation(tf::Quaternion(-geo_quat.y, -geo_quat.z, geo_quat.x, geo_quat.w));
  laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
  tfBoardcaster.sendTransform(laserOdometryTrans);
}

void FeatureAssociation::AdjustOutlierCloud() {
  PointType point;
  for (auto& outlier_point : outlierCloud->points) {
    point.x = outlier_point.y;
    point.y = outlier_point.z;
    point.z = outlier_point.x;
    point.intensity = outlier_point.intensity;
    outlier_point = point;
  }
}

void FeatureAssociation::TransformToEnd(const PointType& pi, PointType& po) {
  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transformCur[0];
  float ry = s * transformCur[1];
  float rz = s * transformCur[2];
  float tx = s * transformCur[3];
  float ty = s * transformCur[4];
  float tz = s * transformCur[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  float x3 = cos(ry) * x2 - sin(ry) * z2;
  float y3 = y2;
  float z3 = sin(ry) * x2 + cos(ry) * z2;

  rx = transformCur[0];
  ry = transformCur[1];
  rz = transformCur[2];
  tx = transformCur[3];
  ty = transformCur[4];
  tz = transformCur[5];

  float x4 = cos(ry) * x3 + sin(ry) * z3;
  float y4 = y3;
  float z4 = -sin(ry) * x3 + cos(ry) * z3;

  float x5 = x4;
  float y5 = cos(rx) * y4 - sin(rx) * z4;
  float z5 = sin(rx) * y4 + cos(rx) * z4;

  float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
  float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
  float z6 = z5 + tz;

  float x7 = cosImuRollStart * (x6 - imuShiftFromStartX) - sinImuRollStart * (y6 - imuShiftFromStartY);
  float y7 = sinImuRollStart * (x6 - imuShiftFromStartX) + cosImuRollStart * (y6 - imuShiftFromStartY);
  float z7 = z6 - imuShiftFromStartZ;

  float x8 = x7;
  float y8 = cosImuPitchStart * y7 - sinImuPitchStart * z7;
  float z8 = sinImuPitchStart * y7 + cosImuPitchStart * z7;

  float x9 = cosImuYawStart * x8 + sinImuYawStart * z8;
  float y9 = y8;
  float z9 = -sinImuYawStart * x8 + cosImuYawStart * z8;

  float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
  float y10 = y9;
  float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

  float x11 = x10;
  float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
  float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

  po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
  po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
  po->z = z11;
  po->intensity = int(pi->intensity);
}

void FeatureAssociation::PublishCloudLast() {
  UpdateIMURollPitchYawStartSinCos();

  for (auto& point : cornerPointsLessSharp->points) {
    TransformToEnd(point, point);
  }

  for (auto& point : surfPointsLessFlat->points) {
    TransformToEnd(point, point);
  }

  std::swap(cornerPointsLessSharp, laserCloudCornerLast);
  std::swap(surfPointsLessFlat, laserCloudSurfLast);

  if (laserCloudCornerLast->points.size() > 10 && laserCloudSurfLast->points.size() > 100) {
    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
  }

  frame_count++;

  if (frame_count >= skip_frame_num + 1) {
    frame_count = 0;

    AdjustOutlierCloud();

    apollo::drivers::PointCloud cloud_msg;
    ToDriverPointCloud(outlierCloud, cloud_msg);
    pub_outlier_cloud_last->Write(cloud_msg);

    cloud_msg->points.clear();
    ToDriverPointCloud(laserCloudCornerLast, cloud_msg);
    pub_laser_cloud_corner_last->Write(cloud_msg);

    cloud_msg->points.clear();
    ToDriverPointCloud(laserCloudSurfLast, cloud_msg);
    pub_laser_cloud_surf_last->Write(cloud_msg);
  }
}

}  // namespace tools
}  // namespace apollo
