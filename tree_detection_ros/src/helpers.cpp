/*
 * helpers.cpp
 *
 *  Created on: Aug 18, 2021
 *      Author: jelavice
 */
#include "tree_detection_ros/helpers.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace tree_detection {

Color::Color() :
		std_msgs::ColorRGBA() {
}
Color::Color(double red, double green, double blue) :
		Color(red, green, blue, 1.0) {
}
Color::Color(double red, double green, double blue, double alpha) :
		Color() {
	r = red;
	g = green;
	b = blue;
	a = alpha;
}

visualization_msgs::Marker getTreeCylinderMarker(const Eigen::Vector3d &treeCoordinates,
		const ClusterDimensions &clusterDimension, int markerId, const ros::Time &timestamp, const std::string &frameId,
		const Color &color) {

	visualization_msgs::Marker treeMarker;

	// Set marker properties
	//this is a debug marker for visualization
	treeMarker.header.frame_id = frameId;
	treeMarker.header.stamp = timestamp;  // ros::Time();
	treeMarker.id = markerId;
	treeMarker.type = visualization_msgs::Marker::CYLINDER;
	treeMarker.action = visualization_msgs::Marker::ADD;
	treeMarker.pose.position.x = treeCoordinates.x();
	treeMarker.pose.position.y = treeCoordinates.y();
	treeMarker.pose.position.z = treeCoordinates.z();
	//todo this just assumes that it will be always pinting upright
	treeMarker.pose.orientation.x = 0.0;
	treeMarker.pose.orientation.y = 0.0;
	treeMarker.pose.orientation.z = 0.0;
	treeMarker.pose.orientation.w = 1.0;
	//get cloud dimensions write a function
	treeMarker.scale.x = clusterDimension.dimX_;
	treeMarker.scale.y = clusterDimension.dimY_;
	treeMarker.scale.z = clusterDimension.dimZ_;
	treeMarker.color.a = color.a;
	treeMarker.color.r = color.r;
	treeMarker.color.g = color.g;
	treeMarker.color.b = color.b;

	return treeMarker;
}

jsk_recognition_msgs::BoundingBox getTreeBBMarker(const Eigen::Vector3d &treeCoordinates,
		const ClusterDimensions &clusterDimension, int markerId, const ros::Time &timestamp, const std::string &frameId) {

	jsk_recognition_msgs::BoundingBox treeMarker;

	// Set marker properties
	//this is a debug marker for visualization
	treeMarker.header.frame_id = frameId;
	treeMarker.header.stamp = timestamp;  // ros::Time();
	treeMarker.pose.position.x = treeCoordinates.x();
	treeMarker.pose.position.y = treeCoordinates.y();
	treeMarker.pose.position.z = treeCoordinates.z();
	//this just assumes that it will be always pinting upright
	treeMarker.pose.orientation.x = 0.0;
	treeMarker.pose.orientation.y = 0.0;
	treeMarker.pose.orientation.z = 0.0;
	treeMarker.pose.orientation.w = 1.0;
	//get cloud dimensions write a function
	treeMarker.dimensions.x = clusterDimension.dimX_ ;
	treeMarker.dimensions.y = clusterDimension.dimY_ ;
	treeMarker.dimensions.z = clusterDimension.dimZ_ ;

	return treeMarker;
}

PointCloud::Ptr fromRos(const sensor_msgs::PointCloud2 &cloud) {
	const PointCloud::Ptr pclTypeCloud(new PointCloud);
	pcl::fromROSMsg(cloud, *pclTypeCloud);
	return pclTypeCloud;
}

sensor_msgs::PointCloud2 toRos(const PointCloud &cloud){
	sensor_msgs::PointCloud2 cloudOut;
	pcl::toROSMsg(cloud, cloudOut);
	return cloudOut;
}

PointCloud::Ptr loadPointcloudFromPcd(const std::string &filename) {
	PointCloud::Ptr cloud(new PointCloud);
	pcl::PCLPointCloud2 cloudBlob;
	pcl::io::loadPCDFile(filename, cloudBlob);
	pcl::fromPCLPointCloud2(cloudBlob, *cloud);
	return cloud;
}

} // namespace tree_detection

