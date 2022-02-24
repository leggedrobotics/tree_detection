/*
 * ElevationMapGroundPlaneRemover.cpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include "point_cloud_preprocessing/PointCloudPreprocessor.hpp"

namespace point_cloud_preprocessing {

    void PointCloudPreprocessor::setParameters(const PointCloudPreprocessorParam &p) {
	    params_ = p;
    }
    const PointCloudPreprocessorParam& PointCloudPreprocessor::getParameters() const {
        return params_;
    }
    void PointCloudPreprocessor::setInputCloudPtr(PointCloud::ConstPtr inputCloud) {
        setInputCloud(*inputCloud);
    }

    void PointCloudPreprocessor::setInputCloud(const PointCloud &inputCloud) {
        inputCloud_.reset(new PointCloud);
        *inputCloud_ = inputCloud; //copy
    }
    void PointCloudPreprocessor::performPreprocessing() {
        preprocessedCloud_.reset(new PointCloud);
        *preprocessedCloud_ = *inputCloud_;
        if (params_.isUseCropBox_) {
            preprocessedCloud_ = applyCropBox(preprocessedCloud_, params_.cropBox_);
        }
        if (params_.isUseVoxelGrid_) {
            preprocessedCloud_ = applyVoxelGrid(preprocessedCloud_, params_.voxelGrid_);
        }
    }

    PointCloud::Ptr PointCloudPreprocessor::applyCropBox(PointCloud::ConstPtr input, const CropBoxParameters &p) const {
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(p.minX_, p.minY_, p.minZ_, 1.0));
        boxFilter.setMax(Eigen::Vector4f(p.maxX_, p.maxY_, p.maxZ_, 1.0));
        boxFilter.setInputCloud(input);
        PointCloud::Ptr outputCloud(new PointCloud());
        boxFilter.filter(*outputCloud);
        return outputCloud;
    }

    PointCloud::Ptr PointCloudPreprocessor::applyVoxelGrid(PointCloud::ConstPtr input, const VoxelGridParameters &p) const {
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(input);
        vox.setLeafSize(p.leafSizeX_, p.leafSizeY_, p.leafSizeZ_);
        PointCloud::Ptr outputCloud(new PointCloud());
        vox.filter(*outputCloud);
	    return outputCloud;
    }

    PointCloud::ConstPtr PointCloudPreprocessor::getPreprocessedCloudPtr() const {
	    return preprocessedCloud_;
    }
    const PointCloud& PointCloudPreprocessor::getPreprocessedCloud() const {
        return *preprocessedCloud_;
    }

}  // namespace point_cloud_preprocessing
