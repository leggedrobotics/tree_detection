/*
 * typedefs.hpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#pragma once
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ground_removal {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointIndices = std::vector<pcl::PointIndices>;
using PointCloudPtrVector = std::vector<PointCloud::Ptr>;


} // tree ground_removal
