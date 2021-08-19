/*
 * creators.cpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#include "tree_detection_ros/creators.hpp"


namespace ground_removal {

void loadParameters(const std::string &filename, ground_removal::GroundPlaneRemoverParam *p) {

	YAML::Node node = YAML::LoadFile(filename);
	loadParameters(node["ground_plane_removal"]["cropbox"], &p->cropBox_);

}

void loadParameters(const std::string &filename, ground_removal::ElevationMapGroundPlaneRemoverParam *p) {

	YAML::Node node = YAML::LoadFile(filename);
	auto groundRemoval = node["ground_plane_removal"]["elevation_map"];
	grid_map::grid_map_pcl::PclLoaderParameters pclLoaderParam;
	pclLoaderParam.loadParameters(groundRemoval);
	p->pclConverter_ = pclLoaderParam;
	std::cout << pclLoaderParam.get().gridMap_.resolution_ << std::endl;

	p->medianFilteringRadius_  = groundRemoval["median_filtering_radius"].as<double>();
	p->medianFilterDownsampleFactor_ = groundRemoval["median_filter_points_downsample_factor"].as<int>();
	p->heightMarginAboeTheSurface_ = groundRemoval["height_margin_above_surface"].as<double>();
	p->isUseMedianFiltering_ = groundRemoval["is_use_median_filter"].as<bool>();
}



std::unique_ptr<ground_removal::GroundPlaneRemover> groundRemoverFactory(const std::string &strategy,
		const std::string &configFile, ros::NodeHandlePtr nh) {

	std::unique_ptr<ground_removal::GroundPlaneRemover> ret;

	if (strategy == "cropbox") {
		ground_removal::GroundPlaneRemoverParam groundPlaneRemovalParam;
		loadParameters(configFile, &groundPlaneRemovalParam);
		auto groundRemover = std::make_unique<ground_removal::GroundPlaneRemover>();
		groundRemover->setParameters(groundPlaneRemovalParam);
		ret = std::move(groundRemover);
	} else if (strategy == "elevation_map") {
		ground_removal::ElevationMapGroundPlaneRemoverParam groundPlaneRemovalParam;
		loadParameters(configFile, &groundPlaneRemovalParam);
		auto groundRemover = std::make_unique<ground_removal::ElevationMapGroundPlaneRemover>(nh.get());
		groundRemover->setParameters(groundPlaneRemovalParam);
		ret = std::move(groundRemover);
	} else {
		throw std::runtime_error("Unknown groun plane removal strategy!!!");
	}

	return ret;
}

} // namespace ground_removal
