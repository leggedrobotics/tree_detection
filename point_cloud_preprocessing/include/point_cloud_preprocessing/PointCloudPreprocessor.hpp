/*
 * elevationMapGroundPlaneRemover.hpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#pragma once
#include "point_cloud_preprocessing/Parameters.hpp"
#include "point_cloud_preprocessing/typedefs.hpp"


namespace point_cloud_preprocessing{


class PointCloudPreprocessor {
public:
	PointCloudPreprocessor() = default;
	virtual ~PointCloudPreprocessor() = default;

    void setParameters(const PointCloudPreprocessorParam &p);
    const PointCloudPreprocessorParam &getParameters() const;

	void setInputCloudPtr(PointCloud::ConstPtr inputCloud);
	void setInputCloud(const PointCloud &inputCloud);

	PointCloud::ConstPtr getPreprocessedCloudPtr() const;
	const PointCloud &getPreprocessedCloud() const;

	void performPreprocessing();

private:
	PointCloud::Ptr applyCropBox(PointCloud::ConstPtr input, const CropBoxParameters &p) const;
	PointCloud::Ptr applyVoxelGrid(PointCloud::ConstPtr input, const VoxelGridParameters &p) const;
	PointCloud::Ptr inputCloud_;
	PointCloud::Ptr preprocessedCloud_;
	PointCloudPreprocessorParam params_;

};


}  // namespace point_cloud_preprocessing
