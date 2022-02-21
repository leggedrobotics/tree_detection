/*
 * Parameters.hpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#pragma once
#include <string>
#include <yaml-cpp/yaml.h>

namespace point_cloud_preprocessing {


struct CropBoxParameters
{
	double minX_ = -50.0;
	double maxX_= 50.0;
	double minY_= -50.0;
	double maxY_= 50.0;
	double minZ_= -10.0;
	double maxZ_= 10.0;
};

struct VoxelGridParameters
{
	double leafSizeX_ = 0.02;
	double leafSizeY_ = 0.02;
	double leafSizeZ_ = 0.02;
};

struct PointCloudPreprocessorParam {
	bool isUseCropBox_ = true;
	CropBoxParameters cropBox_;
	bool isUseVoxelGrid_ = true;
	VoxelGridParameters voxelGrid_;
};

std::ostream& operator<<(std::ostream& out, const PointCloudPreprocessorParam& p);
void loadParameters(const YAML::Node &node, PointCloudPreprocessorParam *p);

} // namespace point_cloud_preprocessing
