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


class PointCloudPreprocessing {
public:
	PointCloudPreprocessing() = default;
	virtual ~PointCloudPreprocessing() = default;

    //  void setParameters(const GroundPlaneRemoverParam &p);
    //  const ElevationMapGroundPlaneRemoverParam &getParameters() const;

private:
	//  ElevationMapGroundPlaneRemoverParam param_;

};


}  // namespace point_cloud_preprocessing
