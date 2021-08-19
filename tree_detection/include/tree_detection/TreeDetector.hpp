#pragma once

// pcl
#include <pcl/PointIndices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tree_detection/common.hpp"
#include "tree_detection/typedefs.hpp"
#include "tree_detection/Parameters.hpp"

namespace tree_detection {
class TreeDetector {
public:
	using ClusterVector = std::vector<Cluster>;

	TreeDetector() = default;
	virtual ~TreeDetector() = default;

	void setTreeDetectionParameters(const TreeDetectionParameters &p);
	void setInputPointCloud(const PointCloud &inputCloud);
	void setInputPointCloudPtr(PointCloud::Ptr inputCloud);
	virtual void detectTrees();
	const ClusterVector& getDetectedTreeClusters() const;
	const TreeDetectionParameters &getParameters() const;
	PointIndices extractClusterIndices(const PointCloud::ConstPtr cloud) const;
private:
	bool isGravityAlignmentStraightEnough(PointCloud::ConstPtr cloudCluster) const;
	double getClusterGravityAlignment(PointCloud::ConstPtr cloudCluster) const;
	bool isClusterTooLow(PointCloud::ConstPtr cloudCluster) const;
	bool isClusterRadiusTooBig(PointCloud::ConstPtr cloudCluster) const;
	PointCloudPtrVector extractValidPointClusters(const PointIndices &indices, PointCloud::ConstPtr cloud) const;
	PointCloud::Ptr extractClusterFromIndices(const std::vector<int> &indices, PointCloud::ConstPtr input) const;
	ClusterDimensions getClusterDimensions(PointCloud::ConstPtr clusterCloud) const;
	ClusterVector extractClusterInformation(const PointCloudPtrVector &validClusters) const;
	Eigen::Vector3d getClusterMean(PointCloud::ConstPtr cloudCluster) const;

	TreeDetectionParameters treeDetectionParam_;
	PointCloud::Ptr inputCloud_;
	ClusterVector detectedTreeClusters_;

};

}  // namespace tree_detection
