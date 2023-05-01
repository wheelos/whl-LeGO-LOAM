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

#include "src/map_optmization.h"

namespace apollo {
namespace tools {

void LoopClosureThread() {
  if (!loopClosureEnableFlag)
    return;

  PerformLoopClosure();
}

bool DetectLoopClosure() {
  latestSurfKeyFrameCloud->clear();
  nearHistorySurfKeyFrameCloud->clear();
  nearHistorySurfKeyFrameCloudDS->clear();

  std::lock_guard<std::mutex> lock(mtx);
  // find the closest history key frame
  std::vector<int> pointSearchIndLoop;
  std::vector<float> pointSearchSqDisLoop;
  kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
  kdtreeHistoryKeyPoses->radiusSearch(currentRobotPosPoint, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

  closestHistoryFrameID = -1;
  for (int i = 0; i < pointSearchIndLoop.size(); ++i) {
    int id = pointSearchIndLoop[i];
    if (abs(cloudKeyPoses6D->points[id].time - timeLaserOdometry) > 30.0) {
      closestHistoryFrameID = id;
      break;
    }
  }
  if (closestHistoryFrameID == -1) {
    return false;
  }
  // save latest key frames
  latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
  *latestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  *latestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);

  pcl::PointCloud<PointType>::Ptr hahaCloud(new pcl::PointCloud<PointType>());
  int cloudSize = latestSurfKeyFrameCloud->points.size();
  for (int i = 0; i < cloudSize; ++i) {
    if ((int)latestSurfKeyFrameCloud->points[i].intensity >= 0) {
      hahaCloud->push_back(latestSurfKeyFrameCloud->points[i]);
    }
  }
  latestSurfKeyFrameCloud->clear();
  *latestSurfKeyFrameCloud = *hahaCloud;
  // save history near key frames
  for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j) {
    if (closestHistoryFrameID + j < 0 || closestHistoryFrameID + j > latestFrameIDLoopCloure)
      continue;
    *nearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[closestHistoryFrameID + j], &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
    *nearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[closestHistoryFrameID + j], &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
  }

  downSizeFilterHistoryKeyFrames.setInputCloud(nearHistorySurfKeyFrameCloud);
  downSizeFilterHistoryKeyFrames.filter(*nearHistorySurfKeyFrameCloudDS);
  // publish history near key frames
  if (pubHistoryKeyFrames.getNumSubscribers() != 0)
  {
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    cloudMsgTemp.header.frame_id = "/camera_init";
    pubHistoryKeyFrames.publish(cloudMsgTemp);
  }

  return true;
}

void PerformLoopClosure() {
  if (cloudKeyPoses3D->points.empty())
    return;
  // try to find close key frame if there are any
  if (potentialLoopFlag == false) {
    if (DetectLoopClosure() == true) {
      potentialLoopFlag = true; // find some key frames that is old enough or close enough for loop closure
      timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
    }
    if (potentialLoopFlag == false)
      return;
  }
  // reset the flag first no matter icp successes or not
  potentialLoopFlag = false;
  // ICP Settings
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  // Align clouds
  icp.setInputSource(latestSurfKeyFrameCloud);
  icp.setInputTarget(nearHistorySurfKeyFrameCloudDS);
  pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
  icp.align(*unused_result);

  if (!icp.hasConverged() || icp.getFitnessScore() > historyKeyframeFitnessScore)
    return;
  // publish corrected cloud
  if (pubIcpKeyFrames.getNumSubscribers() != 0) {
    pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation());
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    cloudMsgTemp.header.frame_id = "/camera_init";
    pubIcpKeyFrames.publish(cloudMsgTemp);
  }

  // get pose constraint
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correctionCameraFrame;
  correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
  pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
  Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch);
  // transform from world origin to wrong pose
  Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  // transform from world origin to corrected pose
  Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
  pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
  gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
  gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[closestHistoryFrameID]);
  gtsam::Vector Vector6(6);
  float noiseScore = icp.getFitnessScore();
  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
  constraintNoise = noiseModel::Diagonal::Variances(Vector6);
  /*
        	add constraints
        	*/
  std::lock_guard<std::mutex> lock(mtx);
  gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID, poseFrom.between(poseTo), constraintNoise));
  isam->update(gtSAMgraph);
  isam->update();
  gtSAMgraph.resize(0);

  aLoopIsClosed = true;
}

void publishGlobalMap() {
  if (pubLaserCloudSurround.getNumSubscribers() == 0)
    return;

  if (cloudKeyPoses3D->points.empty())
    return;
  // kd-tree to find near key frames to visualize
  std::vector<int> pointSearchIndGlobalMap;
  std::vector<float> pointSearchSqDisGlobalMap;
  // search near key frames to visualize
  mtx.lock();
  kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
  kdtreeGlobalMap->radiusSearch(currentRobotPosPoint, globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
  mtx.unlock();

  for (int i = 0; i < pointSearchIndGlobalMap.size(); ++i)
    globalMapKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
  // downsample near selected key frames
  downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
  downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
  // extract visualized and downsampled key frames
  for (int i = 0; i < globalMapKeyPosesDS->points.size(); ++i) {
    int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
    *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    *globalMapKeyFrames += *transformPointCloud(outlierCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
  }
  // downsample visualized points
  downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
  downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

  sensor_msgs::PointCloud2 cloudMsgTemp;
  pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
  cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
  cloudMsgTemp.header.frame_id = "/camera_init";
  pubLaserCloudSurround.publish(cloudMsgTemp);

  globalMapKeyPoses->clear();
  globalMapKeyPosesDS->clear();
  globalMapKeyFrames->clear();
  // globalMapKeyFramesDS->clear();
}

void VisualizeGlobalMapThread() {
  ros::Rate rate(0.2);
  while (ros::ok())
  {
    rate.sleep();
    PublishGlobalMap();
  }
  // save final point cloud
  pcl::io::savePCDFileASCII(fileDirectory + "finalCloud.pcd", *globalMapKeyFramesDS);

  std::string cornerMapString = "/tmp/cornerMap.pcd";
  std::string surfaceMapString = "/tmp/surfaceMap.pcd";
  std::string trajectoryString = "/tmp/trajectory.pcd";

  pcl::PointCloud<PointType>::Ptr cornerMapCloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr cornerMapCloudDS(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surfaceMapCloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surfaceMapCloudDS(new pcl::PointCloud<PointType>());

  for (int i = 0; i < cornerCloudKeyFrames.size(); i++) {
    *cornerMapCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
    *surfaceMapCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
    *surfaceMapCloud += *transformPointCloud(outlierCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
  }

  downSizeFilterCorner.setInputCloud(cornerMapCloud);
  downSizeFilterCorner.filter(*cornerMapCloudDS);
  downSizeFilterSurf.setInputCloud(surfaceMapCloud);
  downSizeFilterSurf.filter(*surfaceMapCloudDS);

  pcl::io::savePCDFileASCII(fileDirectory + "cornerMap.pcd", *cornerMapCloudDS);
  pcl::io::savePCDFileASCII(fileDirectory + "surfaceMap.pcd", *surfaceMapCloudDS);
  pcl::io::savePCDFileASCII(fileDirectory + "trajectory.pcd", *cloudKeyPoses3D);
}

void TransformAssociateToMap() {
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
  transformTobeMapped[0] = -asin(srx);

  float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx) - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly) + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
  float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx) + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly) - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);
  transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                 crycrx / cos(transformTobeMapped[0]));

  float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                 crzcrx / cos(transformTobeMapped[0]));

  x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
  y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  transformTobeMapped[3] = transformAftMapped[3] - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
  transformTobeMapped[4] = transformAftMapped[4] - y2;
  transformTobeMapped[5] = transformAftMapped[5] - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void ExtractSurroundingKeyFrames() {
  if (cloudKeyPoses3D->points.empty())
    return;

  if (loopClosureEnableFlag) {
    // only use recent key poses for graph building
    if (recentCornerCloudKeyFrames.size() < surroundingKeyframeSearchNum)
    { // queue is not full (the beginning of mapping or a loop is just closed)
      // clear recent key frames queue
      recentCornerCloudKeyFrames.clear();
      recentSurfCloudKeyFrames.clear();
      recentOutlierCloudKeyFrames.clear();
      int numPoses = cloudKeyPoses3D->points.size();
      for (int i = numPoses - 1; i >= 0; --i)
      {
        int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
        PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
        updateTransformPointCloudSinCos(&thisTransformation);
        // extract surrounding map
        recentCornerCloudKeyFrames.push_front(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
        recentSurfCloudKeyFrames.push_front(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
        recentOutlierCloudKeyFrames.push_front(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
        if (recentCornerCloudKeyFrames.size() >= surroundingKeyframeSearchNum)
          break;
      }
    }
    else
    { // queue is full, pop the oldest key frame and push the latest key frame
      if (latestFrameID != cloudKeyPoses3D->points.size() - 1)
      { // if the robot is not moving, no need to update recent frames

        recentCornerCloudKeyFrames.pop_front();
        recentSurfCloudKeyFrames.pop_front();
        recentOutlierCloudKeyFrames.pop_front();
        // push latest scan to the end of queue
        latestFrameID = cloudKeyPoses3D->points.size() - 1;
        PointTypePose thisTransformation = cloudKeyPoses6D->points[latestFrameID];
        updateTransformPointCloudSinCos(&thisTransformation);
        recentCornerCloudKeyFrames.push_back(transformPointCloud(cornerCloudKeyFrames[latestFrameID]));
        recentSurfCloudKeyFrames.push_back(transformPointCloud(surfCloudKeyFrames[latestFrameID]));
        recentOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[latestFrameID]));
      }
    }

    for (int i = 0; i < recentCornerCloudKeyFrames.size(); ++i)
    {
      *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
      *laserCloudSurfFromMap += *recentSurfCloudKeyFrames[i];
      *laserCloudSurfFromMap += *recentOutlierCloudKeyFrames[i];
    }
  }
  else
  {
    surroundingKeyPoses->clear();
    surroundingKeyPosesDS->clear();
    // extract all the nearby key poses and downsample them
    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
    kdtreeSurroundingKeyPoses->radiusSearch(currentRobotPosPoint, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis, 0);
    for (int i = 0; i < pointSearchInd.size(); ++i)
      surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    // delete key frames that are not in surrounding region
    int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();
    for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i)
    {
      bool existingFlag = false;
      for (int j = 0; j < numSurroundingPosesDS; ++j)
      {
        if (surroundingExistingKeyPosesID[i] == (int)surroundingKeyPosesDS->points[j].intensity)
        {
          existingFlag = true;
          break;
        }
      }
      if (existingFlag == false)
      {
        surroundingExistingKeyPosesID.erase(surroundingExistingKeyPosesID.begin() + i);
        surroundingCornerCloudKeyFrames.erase(surroundingCornerCloudKeyFrames.begin() + i);
        surroundingSurfCloudKeyFrames.erase(surroundingSurfCloudKeyFrames.begin() + i);
        surroundingOutlierCloudKeyFrames.erase(surroundingOutlierCloudKeyFrames.begin() + i);
        --i;
      }
    }
    // add new key frames that are not in calculated existing key frames
    for (int i = 0; i < numSurroundingPosesDS; ++i)
    {
      bool existingFlag = false;
      for (auto iter = surroundingExistingKeyPosesID.begin(); iter != surroundingExistingKeyPosesID.end(); ++iter)
      {
        if ((*iter) == (int)surroundingKeyPosesDS->points[i].intensity)
        {
          existingFlag = true;
          break;
        }
      }
      if (existingFlag == true)
      {
        continue;
      }
      else
      {
        int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
        PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
        updateTransformPointCloudSinCos(&thisTransformation);
        surroundingExistingKeyPosesID.push_back(thisKeyInd);
        surroundingCornerCloudKeyFrames.push_back(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
        surroundingSurfCloudKeyFrames.push_back(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
        surroundingOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
      }
    }

    for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i)
    {
      *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i];
      *laserCloudSurfFromMap += *surroundingSurfCloudKeyFrames[i];
      *laserCloudSurfFromMap += *surroundingOutlierCloudKeyFrames[i];
    }
  }
  // Downsample the surrounding corner key frames (or map)
  downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
  downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
  laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();
  // Downsample the surrounding surf key frames (or map)
  downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
  downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
  laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
}

void downsampleCurrentScan() {
  laserCloudCornerLastDS->clear();
  downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner.filter(*laserCloudCornerLastDS);
  laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();

  laserCloudSurfLastDS->clear();
  downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf.filter(*laserCloudSurfLastDS);
  laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();

  laserCloudOutlierLastDS->clear();
  downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
  downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
  laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

  laserCloudSurfTotalLast->clear();
  laserCloudSurfTotalLastDS->clear();
  *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
  *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
  downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
  downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
  laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
}

void Scan2MapOptimization() {
  if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100) {
    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

    for (int iterCount = 0; iterCount < 10; iterCount++) {
      laserCloudOri->clear();
      coeffSel->clear();

      cornerOptimization(iterCount);
      surfOptimization(iterCount);

      if (LMOptimization(iterCount) == true)
        break;
    }

    transformUpdate();
  }
}

void saveKeyFramesAndFactor() {
  currentRobotPosPoint.x = transformAftMapped[3];
  currentRobotPosPoint.y = transformAftMapped[4];
  currentRobotPosPoint.z = transformAftMapped[5];

  bool saveThisKeyFrame = true;
  if (sqrt((previousRobotPosPoint.x - currentRobotPosPoint.x) * (previousRobotPosPoint.x - currentRobotPosPoint.x) + (previousRobotPosPoint.y - currentRobotPosPoint.y) * (previousRobotPosPoint.y - currentRobotPosPoint.y) + (previousRobotPosPoint.z - currentRobotPosPoint.z) * (previousRobotPosPoint.z - currentRobotPosPoint.z)) < 0.3)
  {
    saveThisKeyFrame = false;
  }

  if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty())
    return;

  previousRobotPosPoint = currentRobotPosPoint;
  /**
         * update grsam graph
         */
  if (cloudKeyPoses3D->points.empty()) {
    gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]), Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])), priorNoise));
    initialEstimate.insert(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
                                    Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])));
    for (int i = 0; i < 6; ++i)
      transformLast[i] = transformTobeMapped[i];
  } else {
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
                                  Point3(transformLast[5], transformLast[3], transformLast[4]));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
    gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->points.size() - 1, cloudKeyPoses3D->points.size(), poseFrom.between(poseTo), odometryNoise));
    initialEstimate.insert(cloudKeyPoses3D->points.size(), Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                                                 Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
  }
  /**
         * update iSAM
         */
  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  /**
         * save key poses
         */
  PointType thisPose3D;
  PointTypePose thisPose6D;
  Pose3 latestEstimate;

  isamCurrentEstimate = isam->calculateEstimate();
  latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

  thisPose3D.x = latestEstimate.translation().y();
  thisPose3D.y = latestEstimate.translation().z();
  thisPose3D.z = latestEstimate.translation().x();
  thisPose3D.intensity = cloudKeyPoses3D->points.size(); // this can be used as index
  cloudKeyPoses3D->push_back(thisPose3D);

  thisPose6D.x = thisPose3D.x;
  thisPose6D.y = thisPose3D.y;
  thisPose6D.z = thisPose3D.z;
  thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
  thisPose6D.roll = latestEstimate.rotation().pitch();
  thisPose6D.pitch = latestEstimate.rotation().yaw();
  thisPose6D.yaw = latestEstimate.rotation().roll(); // in camera frame
  thisPose6D.time = timeLaserOdometry;
  cloudKeyPoses6D->push_back(thisPose6D);
  /**
         * save updated transform
         */
  if (cloudKeyPoses3D->points.size() > 1) {
    transformAftMapped[0] = latestEstimate.rotation().pitch();
    transformAftMapped[1] = latestEstimate.rotation().yaw();
    transformAftMapped[2] = latestEstimate.rotation().roll();
    transformAftMapped[3] = latestEstimate.translation().y();
    transformAftMapped[4] = latestEstimate.translation().z();
    transformAftMapped[5] = latestEstimate.translation().x();

    for (int i = 0; i < 6; ++i) {
      transformLast[i] = transformAftMapped[i];
      transformTobeMapped[i] = transformAftMapped[i];
    }
  }

  pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(new pcl::PointCloud<PointType>());

  pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
  pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
  pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);

  cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
  surfCloudKeyFrames.push_back(thisSurfKeyFrame);
  outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
}

void correctPoses() {
  if (aLoopIsClosed == true) {
    recentCornerCloudKeyFrames.clear();
    recentSurfCloudKeyFrames.clear();
    recentOutlierCloudKeyFrames.clear();
    // update key poses
    int numPoses = isamCurrentEstimate.size();
    for (int i = 0; i < numPoses; ++i) {
      cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().y();
      cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().z();
      cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().x();

      cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
      cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
      cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
      cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
      cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
      cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
    }

    aLoopIsClosed = false;
  }
}

void publishTF() {
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

  odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
  odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
  odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
  odomAftMapped.pose.pose.orientation.z = geoQuat.x;
  odomAftMapped.pose.pose.orientation.w = geoQuat.w;
  odomAftMapped.pose.pose.position.x = transformAftMapped[3];
  odomAftMapped.pose.pose.position.y = transformAftMapped[4];
  odomAftMapped.pose.pose.position.z = transformAftMapped[5];
  odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
  odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
  odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
  odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
  odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
  odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
  pubOdomAftMapped.publish(odomAftMapped);

  aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
  aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
  tfBroadcaster.sendTransform(aftMappedTrans);
}

void publishKeyPosesAndFrames() {
  if (pubKeyPoses.getNumSubscribers() != 0) {
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    cloudMsgTemp.header.frame_id = "/camera_init";
    pubKeyPoses.publish(cloudMsgTemp);
  }

  if (pubRecentKeyFrames.getNumSubscribers() != 0) {
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    cloudMsgTemp.header.frame_id = "/camera_init";
    pubRecentKeyFrames.publish(cloudMsgTemp);
  }

  if (pubRegisteredCloud.getNumSubscribers() != 0) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
    *cloudOut += *transformPointCloud(laserCloudSurfTotalLast, &thisPose6D);

    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*cloudOut, cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    cloudMsgTemp.header.frame_id = "/camera_init";
    pubRegisteredCloud.publish(cloudMsgTemp);
  }
}

void ClearCloud() {
  laserCloudCornerFromMap->clear();
  laserCloudSurfFromMap->clear();
  laserCloudCornerFromMapDS->clear();
  laserCloudSurfFromMapDS->clear();
}

void Proc() {
  TransformAssociateToMap();
  extractSurroundingKeyFrames();
  downsampleCurrentScan();
  scan2MapOptimization();
  saveKeyFramesAndFactor();
  correctPoses();
  publishTF();
  publishKeyPosesAndFrames();
  ClearCloud();
}

}  // namespace tools
}  // namespace apollo
