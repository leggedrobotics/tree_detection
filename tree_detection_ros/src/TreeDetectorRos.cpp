#include "tree_detection_ros/TreeDetectorRos.hpp"
#include "tree_detection_ros/helpers.hpp"

namespace tree_detection {

TreeDetectorRos::TreeDetectorRos(ros::NodeHandlePtr nh) :
		nh_(nh) {
}

void TreeDetectorRos::initRos(std::string filename) {
	YAML::Node node = YAML::LoadFile(filename);
	auto treeDetectionRos = node["tree_detection_ros"];

	treeBoundingBoxPubliser_ = nh_->advertise<jsk_recognition_msgs::BoundingBoxArray>(treeDetectionRos["tree_bounding_boxes_topic_name"].as<std::string>(), 1, true);
	treeCylinderPublisher_ = nh_->advertise<visualization_msgs::MarkerArray>(treeDetectionRos["tree_cylinders_topic_name"].as<std::string>(), 1, true);
}

void TreeDetectorRos::detectTrees() {

	BASE::detectTrees();
	const auto clusters = getDetectedTreeClusters();
	publishTreeBoundingBoxes(clusters);
	publishTreeCylinderMarkers(clusters);
}

void TreeDetectorRos::setInputPointCloudFromRos(const sensor_msgs::PointCloud2 &cloud) {
	PointCloud::Ptr input = fromRos(cloud);
	setInputPointCloudPtr(input);
	lastCloudInfo_ = cloud.header;
}

void TreeDetectorRos::setCloudFrameId(const std::string &frameId) {
	lastCloudInfo_.frame_id = frameId;
}
void TreeDetectorRos::setCloudTimestamp(const ros::Time &stamp) {
	lastCloudInfo_.stamp = stamp;
}

void TreeDetectorRos::publishTreeCylinderMarkers(const ClusterVector &clusters) const {
	visualization_msgs::MarkerArray markers;
	int id = 0;
	for (const auto &c : clusters) {
		Cluster transformedCluster = c;
		inflateClusterDimensions(&transformedCluster.clusterDimensions_);
		auto marker = getTreeCylinderMarker(transformedCluster.position_, transformedCluster.clusterDimensions_, id++,
				lastCloudInfo_.stamp, lastCloudInfo_.frame_id, Color::Green());
		markers.markers.push_back(marker);
	}
	treeCylinderPublisher_.publish(markers);
}

void TreeDetectorRos::publishTreeBoundingBoxes(const ClusterVector &clusters) const {
	jsk_recognition_msgs::BoundingBoxArray bbxs;
	int id = 0;

	for (const auto &c : clusters) {
		Cluster transformedCluster = c;
		inflateClusterDimensions(&transformedCluster.clusterDimensions_);
		auto bb = getTreeBBMarker(transformedCluster.position_, transformedCluster.clusterDimensions_, id++,
				lastCloudInfo_.stamp, lastCloudInfo_.frame_id);
		bbxs.boxes.push_back(bb);
	}
	bbxs.header.frame_id = lastCloudInfo_.frame_id;
	bbxs.header.stamp = lastCloudInfo_.stamp;
	treeBoundingBoxPubliser_.publish(bbxs);
}

void TreeDetectorRos::inflateClusterDimensions(ClusterDimensions *dim) const {
	const auto &p = getParameters();
	dim->dimX_ *= p.treeMarkerRadiusInflateFactor_;
	dim->dimY_ *= p.treeMarkerRadiusInflateFactor_;
	dim->dimZ_ *= p.treeMarkerHeightInflateFactor_;
}

}  // namespace tree_detection
