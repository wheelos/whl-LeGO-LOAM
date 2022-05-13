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
        for (int j = i - 5; j <= i; ++j) {
          cloud_neighbor_picked[j] = 1;
        }
      } else if (depth2 - depth1 > 0.3) {
        for (int j = i + 1; j <= i + 6; ++j) {
          cloud_neighbor_picked[j] = 1;
        }
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


void ExtractFeatures() {

  for (int i = 0; i < N_SCAN; ++i) {
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

          for (int l = -5; l <= -1; ++l) {
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


void UpdateInitialGuess() {
  Affine3f last_tf = cur_tf;
}


void UpdateTransformation() {
  if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100)
    return;

  for (int iterCount = 0; iterCount < 25; ++iterCount) {
    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingSurfFeatures(iterCount);

    if (laserCloudOri->points.size() < 10)
      continue;
    if (!calculateTransformationSurf(iterCount))
      break;
  }

  for (int iterCount = 0; iterCount < 25; ++iterCount) {
    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingCornerFeatures(iterCount);

    if (laserCloudOri->points.size() < 10)
      continue;
    if (!calculateTransformationCorner(iterCount))
      break;
  }
}


void FindCorrespondingSurfFeatures(int iterCount) {
  int surfPointsFlatNum = surfPointsFlat->points.size();

  for (int i = 0; i < surfPointsFlatNum; ++i) {
    TransformToStart(&surfPointsFlat->points[i], &pointSel);

    if (iterCount % 5 == 0) {
      kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

      if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
        for (int j = closestPointInd + 1; j < cornerPointsSharpNum; ++j) {
          int index = int(laserCloudCornerLast->points[j].intensity);
          if (index > closestPointScan + 2.5) {
            break;
          }
          pointSqDis = pcl::squaredEuclideanDistance(laserCloudCornerLast->points[j], pointSel);
          if (index > closestPointScan && pointSqDis < minPointSqDis2) {
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
          }
        }

        for (int j = closestPointInd - 1; j >= 0; --j) {
          int index = int(laserCloudCornerLast->points[j].intensity);
          if (index < closestPointScan - 2.5) {
            break;
          }
          pointSqDis = pcl::squaredEuclideanDistance(laserCloudCornerLast->points[j], pointSel);
          if (index < closestPointScan && pointSqDis < minPointSqDis2) {
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
          }
        }
      }

      pointSearchCornerInd1[i] = closestPointInd;
      pointSearchCornerInd2[i] = minPointInd2;
    } // iterCount % 5 == 0

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

      float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
      float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
      float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

      float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

      float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

      float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

      float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

      float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

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
    } // pointSearchCornerInd2[i]
  } // for surfPointsFlatNum
}

bool CalculateTransformationSurf(int iterCount) {

}

bool CalculateTransformationCorner(int iterCount) {

}


void FindCorrespondingCornerFeatures(int iterCount) {
  int cornerPointsSharpNum = cornerPointsSharp->points.size();
  for (int i = 0; i < cornerPointsSharpNum; ++i) {
    TransformToStart(&cornerPointsSharp->points[i], &pointSel);

    if (iterCount % 5 == 0) {
      kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

      if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
        for (int j = closestPointInd + 1; j < cornerPointsSharpNum; ++j) {
          int index = int(laserCloudCornerLast->points[j].intensity);
          if (index > closestPointScan + 2.5) {
            break;
          }

          pointSqDis = pcl::squaredEuclideanDistance(laserCloudCornerLast->points[j], pointSel);

          if (index > closestPointScan && pointSqDis < minPointSqDis2) {
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
          }
        }

        for (int j = closestPointInd - 1; j >= 0; --j) {
          int index = int(laserCloudCornerLast->points[j].intensity);
          if (index < closestPointScan - 2.5) {
            break;
          }

          pointSqDis = pcl::squaredEuclideanDistance(laserCloudCornerLast->points[j], pointSel);

          if (index < closestPointScan && pointSqDis < minPointSqDis2) {
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
          }
        }
      } // nearestFeatureSearchSqDist

      pointSearchCornerInd1[i] = closestPointInd;
      pointSearchCornerInd2[i] = minPointInd2;
    } // iterCount

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

      float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
      float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
      float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

      float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

      float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

      float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

      float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

      float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

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
    } // pointSearchCornerInd2[i]
  } // for cornerPointsSharpNum
}


}  // namespace tools
}  // namespace apollo
