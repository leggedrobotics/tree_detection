/*
 * common.hpp
 *
 *  Created on: Aug 18, 2021
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Core>

namespace tree_detection {

struct ClusterDimensions
{
  double dimX_;
  double dimY_;
  double dimZ_;
};

struct Cluster
{
  Eigen::Vector3d position_;
  ClusterDimensions clusterDimensions_;
};

struct RPY
{
  double roll_;
  double pitch_;
  double yaw_;
};


} // tree detection
