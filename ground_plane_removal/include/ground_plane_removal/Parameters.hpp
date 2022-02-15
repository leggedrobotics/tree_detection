/*
 * Parameters.hpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include "grid_map_pcl/PclLoaderParameters.hpp"

namespace ground_removal {


struct GroundPlaneCropBoxParameters
{
  double minX_ = -50.0;
  double maxX_= 50.0;
  double minY_= -50.0;
  double maxY_= 50.0;
  double minZ_= -10.0;
  double maxZ_= 10.0;
};

struct GroundPlaneVoxelGridParameters
{
  double leafSizeX_ = 0.02;
  double leafSizeY_ = 0.02;
  double leafSizeZ_ = 0.02;
};


struct GroundPlaneRemoverParam {
	bool isUseVoxelGrid_ = true;
	GroundPlaneCropBoxParameters cropBox_;
	GroundPlaneVoxelGridParameters voxelGrid_;
};

struct ElevationMapGroundPlaneRemoverParam : public GroundPlaneRemoverParam {
	grid_map::grid_map_pcl::PclLoaderParameters pclConverter_;
	double minHeightAboveGround_ = 0.1;
	double maxHeightAboveGround_ = 8.0;
	double medianFilteringRadius_ = 2.0;
	int medianFilterDownsampleFactor_  = 1;
	bool isUseMedianFiltering_ = true;
	bool isUseCropBox_ = true;
};

std::ostream& operator<<(std::ostream& out, const GroundPlaneCropBoxParameters& p);
void loadParameters(const YAML::Node &node, GroundPlaneCropBoxParameters *p);

std::ostream& operator<<(std::ostream& out, const GroundPlaneVoxelGridParameters& p);
void loadParameters(const YAML::Node &node, GroundPlaneVoxelGridParameters *p);

} // namespace ground_removal
