/*
 * Parameters.cpp
 *
 *  Created on: Aug 18, 2021
 *      Author: jelavice
 */

#include "tree_detection/Parameters.hpp"

namespace tree_detection {

void loadParameters(const YAML::Node &node, TreeDetectionParameters *p) {
	p->maxDiameterTree_ = node["max_diameter"].as<double>();
	p->minHeightTree_ = node["min_height"].as<double>();
	p->clusterTolerance_ = node["cluster_tolerance"].as<double>();
	p->minNumPointsTree_ = node["min_num_points"].as<double>();
	p->maxNumPointsTree_ = node["max_num_points"].as<double>();
	p->minEigenVectorAlignment_ = node["min_eigenvector_alignment"].as<double>();
	p->treeMarkerHeightInflateFactor_ = node["tree_marker_height_inflate_factor"].as<double>();
	p->treeMarkerRadiusInflateFactor_ = node["tree_marker_radius_inflate_factor"].as<double>();
	p->treeMarkerOpacity_ = node["tree_marker_opacity"].as<double>();
	p->isPrintDiscardedTreeInstances_ = node["is_print_discarded_clusters"].as<bool>();
	p->isPrintTiming_ = node["is_print_timing"].as<bool>();
}
void loadParameters(const std::string &filename, TreeDetectionParameters *p) {
	YAML::Node node = YAML::LoadFile(filename);

	if (node.IsNull()) {
		throw std::runtime_error("Tree cloud parameters loading failed");
	}
	loadParameters(node["tree_detection"], p);
}

void loadParameters(const YAML::Node &node, CloudCroppingParameters *p) {
	p->cropBoxMinX_ = node["crop_box_minX"].as<double>();
	p->cropBoxMaxX_ = node["crop_box_maxX"].as<double>();
	p->cropBoxMinY_ = node["crop_box_minY"].as<double>();
	p->cropBoxMaxY_ = node["crop_box_maxY"].as<double>();
	p->cropBoxMinZ_ = node["crop_box_minZ"].as<double>();
	p->cropBoxMaxZ_ = node["crop_box_maxZ"].as<double>();
}
void loadParameters(const std::string &filename, CloudCroppingParameters *p) {
	YAML::Node node = YAML::LoadFile(filename);

	if (node.IsNull()) {
		throw std::runtime_error("Tree cloud parameters loading failed");
	}
	loadParameters(node["cropping"], p);
}

} // namespace tree_detection
