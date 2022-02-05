#pragma once

#include <ros/ros.h>
#include "tree_detection/TreeDetector.hpp"
#include "sensor_msgs/PointCloud2.h"

namespace tree_detection {

class TreeDetectorRos: public TreeDetector {
	using BASE = TreeDetector;
public:
	explicit TreeDetectorRos(ros::NodeHandlePtr nh);

	void initRos(std::string filename);
	void detectTrees() override;
	void setInputPointCloudFromRos(const sensor_msgs::PointCloud2 &cloud);
	void publishTreeCylinderMarkers(const ClusterVector &clusters) const;
	void publishTreeBoundingBoxes(const ClusterVector &clusters) const;
	void setCloudFrameId(const std::string &frameId);
	void setCloudTimestamp(const ros::Time &stamp);
private:

	void inflateClusterDimensions(ClusterDimensions *dim) const;

	ros::Publisher treeBoundingBoxPubliser_;
	ros::Publisher treeCylinderPublisher_;
	ros::NodeHandlePtr nh_;
	std_msgs::Header lastCloudInfo_;

};

}  // namespace tree_detection
