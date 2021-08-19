/*
 * GroundPlaneRemover.cpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#include "ground_plane_removal/GroundPlaneRemover.hpp"
#include <pcl/filters/crop_box.h>


namespace ground_removal {

void GroundPlaneRemover::setParameters(const GroundPlaneRemoverParam &p) {
	params_ = p;
}
const GroundPlaneRemoverParam& GroundPlaneRemover::getParameters() const {
	return params_;
}
void GroundPlaneRemover::setInputCloudPtr(PointCloud::ConstPtr inputCloud) {
	setInputCloud(*inputCloud);
}

void GroundPlaneRemover::setInputCloud(const PointCloud &inputCloud) {
	inputCloud_.reset(new PointCloud);
	*inputCloud_ = inputCloud; //copy
}
void GroundPlaneRemover::removeGroundPlane() {
	noGroundPlaneCloud_ = applyCropBox(inputCloud_, params_.cropBox_);
}

PointCloud::Ptr GroundPlaneRemover::applyCropBox(PointCloud::ConstPtr input, const GroundPlaneCropBoxParameters &p) const {
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	  boxFilter.setMin(Eigen::Vector4f(p.minX_, p.minY_, p.minZ_, 1.0));
	  boxFilter.setMax(Eigen::Vector4f(p.maxX_, p.maxY_, p.maxZ_, 1.0));
	  boxFilter.setInputCloud(input);
	  PointCloud::Ptr outputCloud(new PointCloud());
	  boxFilter.filter(*outputCloud);
	  return outputCloud;
}

PointCloud::ConstPtr GroundPlaneRemover::getCloudWithoutGroundPlanePtr() const {
	return noGroundPlaneCloud_;
}
const PointCloud& GroundPlaneRemover::getCloudWithoutGroundPlane() const {
	return *noGroundPlaneCloud_;
}

}  // namespace ground_removal

