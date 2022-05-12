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

#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>



#include "cyber/cyber.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/tools/ilego_loam/proto/cloud_info.pb.h"


namespace apollo {
namespace tools {

class ImageProjection final : public cyber::Component<> {
 public:
  ImageProjection();
  ~ImageProjection();

  bool Init() override;

 private:
  void CloudHandler(const DriverPointCloudPtr& laser_cloud_msg);

  void CopyPointCloud(const DriverPointCloudPtr& laser_cloud_msg);
  void FindStartEndAngle();
  void ProjectPointCloud();
  void GroundRemoval();
  void CloudSegmentation();
  void PublishCloud();
  void ResetParameters();

  void LabelComponents(int row, int col);

  void AllocateMemory();

 private:
  std::shared_ptr<cyber::Reader<apollo::drivers::PointCloud>> sub_laser_cloud;

  std::shared_ptr<cyber::Writer<apollo::drivers::PointCloud>> pub_full_cloud;
  std::shared_ptr<cyber::Writer<apollo::drivers::PointCloud>> pub_full_info_cloud;
  std::shared_ptr<cyber::Writer<apollo::drivers::PointCloud>> pub_ground_cloud;
  std::shared_ptr<cyber::Writer<apollo::drivers::PointCloud>> pub_segmented_cloud;
  std::shared_ptr<cyber::Writer<apollo::drivers::PointCloud>> pub_segmented_cloud_pure;
  std::shared_ptr<cyber::Writer<cloud_msgs::CloudInfo>> pub_segmented_cloud_info;
  std::shared_ptr<cyber::Writer<apollo::drivers::PointCloud>> pub_outlier_cloud;

  PointCloudPtr laser_cloud_in;
  // PointCloudRPtr laser_cloud_in_ring;
  PointCloudPtr laser_cloud_in_ring;

  PointCloudPtr full_cloud;
  PointCloudPtr full_info_cloud;

  PointCloudPtr ground_cloud;
  PointCloudPtr segmented_cloud;
  PointCloudPtr segmented_cloud_pure;
  PointCloudPtr outlier_cloud;

  pcl::PointXYZI nan_point;

  cv::Mat range_mat;
  cv::Mat label_mat;
  cv::Mat ground_mat;

  int label_count;

  cloud_msgs::CloudInfo seg_msg;
  apollo::common::Header cloud_header;

  std::vector<uint16_t> all_pushed_indX;
  std::vector<uint16_t> all_pushed_indY;

  std::vector<uint16_t> queue_indX;
  std::vector<uint16_t> queue_indY;
};

CYBER_REGISTER_COMPONENT(ImageProjection)

}  // namespace tools
}  // namespace apollo
