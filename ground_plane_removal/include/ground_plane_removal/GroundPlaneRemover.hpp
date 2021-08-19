/*
 * GroundPlaneRemover.hpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#pragma once
#include "ground_plane_removal/Parameters.hpp"
#include "ground_plane_removal/typedefs.hpp"



namespace ground_removal{


class GroundPlaneRemover {
public:
	GroundPlaneRemover() = default;
	virtual ~GroundPlaneRemover() = default;


	virtual void setParameters(const GroundPlaneRemoverParam &p);
	virtual const GroundPlaneRemoverParam &getParameters() const;
	void setInputCloudPtr(PointCloud::ConstPtr inputCloud);
	void setInputCloud(const PointCloud &inputCloud);
	virtual void removeGroundPlane();
	PointCloud::ConstPtr getCloudWithoutGroundPlanePtr() const;
	const PointCloud &getCloudWithoutGroundPlane() const;

protected:
	PointCloud::Ptr applyCropBox(PointCloud::ConstPtr input, const GroundPlaneCropBoxParameters &p) const;
	PointCloud::Ptr inputCloud_;
	PointCloud::Ptr noGroundPlaneCloud_;
private:

	GroundPlaneRemoverParam params_;




};


}  // namespace ground_removal
