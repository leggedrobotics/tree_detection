/*
 * Parameters.hpp
 *
 *  Created on: Aug 18, 2021
 *      Author: jelavice
 */

#pragma once
#include <string>
#include <yaml-cpp/yaml.h>

namespace tree_detection {

struct TreeDetectionParameters {
	double maxDiameterTree_ = 0.6;
	double minHeightTree_ = 0.5;
	double clusterTolerance_ = 0.3;
	int minNumPointsTree_ = 10;
	int maxNumPointsTree_ = 1000;
	double minEigenVectorAlignment_ = 0.8;
	double treeMarkerRadiusInflateFactor_ = 1.0;
	double treeMarkerHeightInflateFactor_ = 1.0;
	double treeMarkerOpacity_ = 1.0;
	bool isPrintDiscardedTreeInstances_ = false;
	bool isPrintTiming_ = false;
};

struct CloudCroppingParameters {
	double cropBoxMinX_ = -50.0;
	double cropBoxMaxX_ = 50.0;
	double cropBoxMinY_ = -50.0;
	double cropBoxMaxY_ = 50.0;
	double cropBoxMinZ_ = -10.0;
	double cropBoxMaxZ_ = 20.0;
};

void loadParameters(const YAML::Node &node, TreeDetectionParameters *p);
void loadParameters(const std::string &filename, TreeDetectionParameters *p);
void loadParameters(const YAML::Node &node, CloudCroppingParameters *p);
void loadParameters(const std::string &filename, CloudCroppingParameters *p);

} // namespace tree_detection
