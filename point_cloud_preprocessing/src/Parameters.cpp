/*
 * Parameters.cpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */
#include "point_cloud_preprocessing/Parameters.hpp"
#include <iomanip>

namespace point_cloud_preprocessing {

void loadParameters(const std::string &filename, PointCloudPreprocessorParam *p) {
	YAML::Node node = YAML::LoadFile(filename);

	if (node.IsNull()) {
		throw std::runtime_error("Point cloud preprocessing parameters loading failed");
	}
	loadParameters(node["point_cloud_preprocessing"], p);
}

void loadParameters(const YAML::Node &node, PointCloudPreprocessorParam *p) {
		p->isUseCropBox_ = node["is_use_crop_box"].as<bool>();
		p->cropBox_.minX_ = node["crop_box"]["crop_box_minX"].as<double>();
		p->cropBox_.maxX_ = node["crop_box"]["crop_box_maxX"].as<double>();
		p->cropBox_.minY_ = node["crop_box"]["crop_box_minY"].as<double>();
		p->cropBox_.maxY_ = node["crop_box"]["crop_box_maxY"].as<double>();
		p->cropBox_.minZ_ = node["crop_box"]["crop_box_minZ"].as<double>();
		p->cropBox_.maxZ_ = node["crop_box"]["crop_box_maxZ"].as<double>();

		p->isUseVoxelGrid_ = node["is_use_voxel_grid"].as<bool>();
		p->voxelGrid_.leafSizeX_ = node["voxel_grid"]["leaf_size_x"].as<double>();
		p->voxelGrid_.leafSizeY_ = node["voxel_grid"]["leaf_size_y"].as<double>();
		p->voxelGrid_.leafSizeZ_ = node["voxel_grid"]["leaf_size_z"].as<double>();
	}

std::ostream& operator<<(std::ostream& out, const PointCloudPreprocessorParam& p) {
		out << "┌────────────────────────────────────────────────────┐\n";
		out << "│             Point Cloud Preprocessor Param         │\n";
		out << "├─────────────────────────────────┬──────────────────┤\n";
		out << "│ " << std::left << std::setw(31) << "is use crop box"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.isUseCropBox_  << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";
		out << "│ " << std::left << std::setw(31) << "crop box minX"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.cropBox_.minX_  << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";
		out << "│ " << std::left << std::setw(31) << "crop box max X"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.cropBox_.minX_ << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";
		out << "│ " << std::left << std::setw(31) << "crop box min Y"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.cropBox_.minY_ << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";
		out << "│ " << std::left << std::setw(31) << "crop box max Y"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.cropBox_.maxY_ << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";
		out << "│ " << std::left << std::setw(31) << "crop box min Z"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.cropBox_.minZ_ << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";
		out << "│ " << std::left << std::setw(31) << "crop box max Z"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.cropBox_.maxZ_ << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";
		out << "│ " << std::left << std::setw(31) << "is use voxel grid"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.isUseVoxelGrid_  << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";
		out << "│ " << std::left << std::setw(31) << "voxel grid leaf size X "
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.voxelGrid_.leafSizeX_ << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";

		out << "│ " << std::left << std::setw(31) << "voxel grid leaf size Y"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.voxelGrid_.leafSizeY_ << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";

		out << "│ " << std::left << std::setw(31) << "voxel grid leaf size Z"
				<< " │ " << std::right << std::setw(16) << std::fixed
				<< std::setprecision(4) << p.voxelGrid_.leafSizeZ_ << " │\n";
		out << "├─────────────────────────────────┼──────────────────┤\n";

  return out;
}

} // namespace point_cloud_preprocessing



