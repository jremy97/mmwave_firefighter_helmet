//
// Created by doug on 2021-03-31.
//

#ifndef SRC_POINT_MATCHER_H
#define SRC_POINT_MATCHER_H

#include <iostream>

#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/correspondence.h>

#include <vector>
#include <algorithm>

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>
#include <future>

#define pointXYZIVector pcl::PointCloud<pcl::PointXYZI>::Ptr

const double CORRESPONDENCE_TOLERANCE = 0.05;

//void matchPoints(pointXYZIVector previous, pointXYZIVector current);
Eigen::Matrix4f maximumCorrespondenceTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr existingCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud);

#endif //SRC_POINT_MATCHER_H
